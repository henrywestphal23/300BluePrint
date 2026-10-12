#include "Arduino.h"
#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

#define EMMISIVITY 0.97 //how well an object reflects thermal radiation
#define TA_SHIFT 8 //ambient temperature of the device?

#define GRID_W            32
#define GRID_H            24
#define NUM_PIXELS        (GRID_W * GRID_H)

#define WARM_DELTA        3.0f   // deg C above background to count as "person"
#define BG_ALPHA          0.01f  // background adaptation rate (non-person pixels)
#define MIN_BLOB_PIXELS   50     // reject blobs smaller than this (noise)
#define MAX_BLOBS         6
#define MAX_TRACKS        6
#define MAX_MATCH_DIST    14.0f  // max pixels a person can move between frames
#define MAX_MISSED_FRAMES 4      // frames a track can vanish before it's finished
#define MIN_TRAVEL_PX     5.0f   // net x-movement needed to count as a pass

paramsMLX90640 mlx90640;
const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640
static float tempValues[32 * 24];

enum Direction {DIR_NONE, DIR_IN, DIR_OUT};

struct Blob {
  float cx, cy;
  int area;
  int max_area = 0;
};

struct Track {
  bool active;
  uint8_t id;
  float iX, iY;
  float x, y;
  uint8_t missed;
  int max_area;
};

static float background[NUM_PIXELS];
static bool backgroundReady = false;
static bool foreground[NUM_PIXELS];

static Blob blobs[MAX_BLOBS];
static Track tracks[MAX_TRACKS];
static uint8_t nextTrackId = 1;

static int peopleTotal = 0;
static int peopleIn = 0;
static int peopleOut = 0;

void readTempValues() {
  for (byte x = 0; x < 2; x++) {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0) {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
    float tr = Ta - TA_SHIFT;
    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, EMMISIVITY, tr, tempValues);
  }
}

// ------------------------- foreground mask ---------------------------------

void updateBackgroundAndMask() { //distinguishes foreground if its >3C above background
  if (!backgroundReady) {
    for (int i = 0; i < NUM_PIXELS; i++) 
    background[i] = tempValues[i];
    backgroundReady = true;
  }
  for (int i = 0; i < NUM_PIXELS; i++) {
    foreground[i] = (tempValues[i] - background[i]) > WARM_DELTA;
    if (!foreground[i]) {
      // only adapt on non-person pixels so a person doesn't get absorbed
      background[i] += BG_ALPHA * (tempValues[i] - background[i]); //no clue, increases background by value?
    }
  }
}

// ------------------------- blob detection ----------------------------------
// Connected-component search (4-connectivity) using an explicit stack.
// Returns number of blobs found; results are in blobs[].

int findBlobs() {
  static bool visited[NUM_PIXELS];
  static int16_t stack[NUM_PIXELS];
  memset(visited, 0, sizeof(visited));
  int count = 0;
  blobs[count].max_area = 0;
  for (int i = 0; i < NUM_PIXELS && count < MAX_BLOBS; i++) {
    if (!foreground[i] || visited[i]) continue;

    int sp = 0;
    stack[sp++] = i;
    visited[i] = true;
    long sumX = 0, sumY = 0;
    int area = 0;

    while (sp > 0) { //counts connected pixels
      int p = stack[--sp];
      int px = p % GRID_W;
      int py = p / GRID_W;
      sumX += px;
      sumY += py;
      area++;

      const int dx[4] = {1, -1, 0, 0};
      const int dy[4] = {0, 0, 1, -1};
      for (int k = 0; k < 4; k++) {
        int nx = px + dx[k];
        int ny = py + dy[k];
        if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H) continue;
        int q = ny * GRID_W + nx;
        if (foreground[q] && !visited[q]) {
          visited[q] = true;
          stack[sp++] = q;
        }
      }
    }

    if (area >= MIN_BLOB_PIXELS) {
      blobs[count].cx = (float)sumX / area;
      blobs[count].cy = (float)sumY / area;
      blobs[count].area = area;

      //Debug code
      Serial.print("blob # and area: ");
      Serial.print(count);
      Serial.print(" | ");
      Serial.println(area);
      Serial.println();

      if (blobs[count].max_area < area) {
        blobs[count].max_area = area;
      }
      count++;
    }
  }
  return count;
}

// ------------------------- tracking ----------------------------------------

void updateTracks(int numBlobs) {
  bool blobUsed[MAX_BLOBS] = {false};
  bool trackMatched[MAX_TRACKS] = {false};

  // Match each active track to its nearest unused blob
  for (int t = 0; t < MAX_TRACKS; t++) {
    if (!tracks[t].active) continue;
    float best = MAX_MATCH_DIST;
    int bestB = -1;

    for (int b = 0; b < numBlobs; b++) { //iterates through blobs and chooses closest blob
      if (blobUsed[b]) continue;
      float dx = blobs[b].cx - tracks[t].x;
      float dy = blobs[b].cy - tracks[t].y;
      float d = sqrtf(dx * dx + dy * dy);
      if (d < best) { 
        best = d; 
        bestB = b;
      }
    }

    if (bestB >= 0) {
      tracks[t].x = blobs[bestB].cx;
      tracks[t].y = blobs[bestB].cy;
      tracks[t].missed = 0;
      blobUsed[bestB] = true;
      trackMatched[t] = true;
    }
  }

  // Tracks with no match: age them, finish them if gone too long
  for (int t = 0; t < MAX_TRACKS; t++) {
    if (!tracks[t].active || trackMatched[t]) continue;
    tracks[t].missed++;
    if (tracks[t].missed > MAX_MISSED_FRAMES) {
      finishTrack(tracks[t]);
      tracks[t].active = false;
      
      Serial.print("track killed: ");
      Serial.println(track[t].id);
      Serial.println();
    }
  }

  // Unmatched blobs become new tracks
  for (int b = 0; b < numBlobs; b++) {
    if (blobUsed[b]) continue;
    for (int t = 0; t < MAX_TRACKS; t++) {
      if (tracks[t].active) {
        Serial.print("tracks | iX | x: ");
        Serial.print(tracks[t].id);
        Serial.print(" | ");
        Serial.print(tracks[t].iX);
        Serial.print(" | ");
        Serial.println(tracks[t].x);
        Serial.println();
      }
      if (!tracks[t].active) {
        tracks[t].active = true;
        tracks[t].id = nextTrackId++;
        tracks[t].iX = tracks[t].x = blobs[b].cx;
        tracks[t].iY = tracks[t].y = blobs[b].cy;
        tracks[t].missed = 0;

        tracks[t].max_area = blobs[b].max_area;

        break;
      }
    }
  }
}

// ------------------------- direction function ------------------------------
// Net movement along x from where the track first appeared to where it is now.

Direction getTrackDirection(const Track &t) {
  float dx = t.x - t.iX;
  if (abs(dx) < MIN_TRAVEL_PX) return DIR_NONE;  // didn't really cross
  return (dx > 0) ? DIR_IN : DIR_OUT;            // flip if mounted the other way
}

// ------------------------- counting function -------------------------------
// Called when a track ends. Counts it as a person only if it actually passed
// through (direction != NONE). Returns the running total.

void countPeople(const Track &t) {
  Direction d = getTrackDirection(t);
  if (d == DIR_NONE);
  else if (d == DIR_IN) 
    peopleIn++; 
  else 
    peopleOut++;
  return;
}

void finishTrack(const Track &t) {
  Direction d = getTrackDirection(t);
  countPeople(t);
  if (d != DIR_NONE) {

    Serial.print("blobs max area: ");
    Serial.println(t.max_area);

    Serial.print(d == DIR_IN ? "Went IN" : "Went OUT");
    Serial.print(" | total: ");
    Serial.print(max(peopleIn - peopleOut, 0));
    Serial.print(" in: ");
    Serial.print(peopleIn);
    Serial.print(" out: ");
    Serial.println(peopleOut);
  }
}

//-------------------------------------------------------------------------------------------------//
void Device_Scan() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0) {
    Serial.println("MLX90640 not detected at default I2C address. iing scan the device addr...");
    Device_Scan();

  }
  else {
    Serial.println("MLX90640 online!");
  }
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0) Serial.println("Failed to load system parameters");
  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0) Serial.println("Parameter extraction failed");
  MLX90640_SetRefreshRate(MLX90640_address, 0x05); 
  Wire.setClock(800000);
}

void loop() {
  readTempValues();               // fills tempValues[]
  updateBackgroundAndMask();      // fills foreground[]
  int n = findBlobs();            // fills blobs[]

  updateTracks(n);                // matches blobs to tracks, finishes old ones
  delay(30);
}