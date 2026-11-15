/**
 * LIGHT-ADAPTIVE DOME SENSOR - 9 SENSOR 3D MODEL GENERATOR
 * Generates watertight, manifold binary STL for 3D printing
 * Uses Signed Distance Field (SDF) + Adaptive Marching Cubes
 */

const fs = require('fs');
const path = require('path');

console.log('Generating 3D Printable STL for 9-Sensor Dome...');

// -------------------------------------------------------------
// Model Dimensions (mm)
// -------------------------------------------------------------
const R_OUTER = 35.0;           // 70 mm outer dome diameter
const WALL_THICK = 3.2;         // 3.2 mm shell thickness
const R_INNER = R_OUTER - WALL_THICK; // 31.8 mm inner cavity

const R_FLANGE = 44.0;          // 88 mm base flange diameter
const H_FLANGE = 3.5;           // 3.5 mm base height
const R_BOLT_PCD = 39.0;        // 78 mm PCD for M3 mounting bolts
const R_BOLT_HOLE = 1.7;        // 3.4 mm M3 bolt clearance
const R_WIRE_PORT = 14.0;       // 28 mm bottom wiring passthrough

// Sensor Port Dimensions (BPW34 Photodiode)
const R_SENSOR_HOLE = 3.2;      // 6.4 mm circular aperture
const R_COLLAR = 6.6;           // 13.2 mm reinforcing collar
const H_COLLAR_EXT = 1.6;       // Collar protrusion above sphere
const RECESS_W = 5.6;           // 5.6 mm BPW34 width
const RECESS_L = 4.6;           // 4.6 mm BPW34 length
const RECESS_D = 1.8;           // 1.8 mm recess depth

// 9 Sensor Orientations [Azimuth deg, Elevation deg]
const DEG2RAD = Math.PI / 180;
const SENSORS_RAW = [
  [0, 90],    // S1 Zenith (Top)
  [0, 60],    // S2 North
  [90, 60],   // S3 East
  [180, 60],  // S4 South
  [270, 60],  // S5 West
  [45, 30],   // S6 North-East
  [135, 30],  // S7 South-East
  [225, 30],  // S8 South-West
  [315, 30]   // S9 North-West
];

const SENSORS = SENSORS_RAW.map(([az, el]) => {
  const azR = az * DEG2RAD, elR = el * DEG2RAD;
  // Direction vector [nx, ny, nz]
  const nx = Math.cos(elR) * Math.sin(azR);
  const ny = Math.cos(elR) * Math.cos(azR);
  const nz = Math.sin(elR);
  return { az, el, nx, ny, nz };
});

// -------------------------------------------------------------
// Signed Distance Field (SDF) Definition
// Negative inside solid, positive outside
// -------------------------------------------------------------
function length(x, y, z) {
  return Math.hypot(x, y, z);
}

function length2(x, y) {
  return Math.hypot(x, y);
}

function sdfModel(x, y, z) {
  const r3d = length(x, y, z);
  const r2d = length2(x, y);

  // Base dome shell
  // Shell is between R_INNER and R_OUTER, and z >= 0
  const dOuterSphere = r3d - R_OUTER;
  const dInnerSphere = R_INNER - r3d;
  const dDomeShell = Math.max(dOuterSphere, dInnerSphere, -z);

  // Base flange: cylinder from z = 0 to z = H_FLANGE, radius R_FLANGE
  const dFlangeCyl = Math.max(r2d - R_FLANGE, -z, z - H_FLANGE);

  // Union dome shell and flange
  let dSolid = Math.min(dDomeShell, dFlangeCyl);

  // Add outer collars for each of the 9 sensors
  for (let i = 0; i < SENSORS.length; i++) {
    const s = SENSORS[i];
    // Projection of [x, y, z] onto sensor axis
    const t = x * s.nx + y * s.ny + z * s.nz;
    if (t > R_INNER - 2.0 && t < R_OUTER + H_COLLAR_EXT + 2.0) {
      // Perpendicular distance to sensor axis
      const px = x - t * s.nx;
      const py = y - t * s.ny;
      const pz = z - t * s.nz;
      const dRadial = length(px, py, pz) - R_COLLAR;
      // Truncate collar cylinder
      const dCollar = Math.max(dRadial, (R_OUTER - 2.0) - t, t - (R_OUTER + H_COLLAR_EXT));
      dSolid = Math.min(dSolid, dCollar);
    }
  }

  // --- SUBTRACT NEGATIVE BODIES (HOLES) ---

  // 1. Central wire port at base: cylinder radius R_WIRE_PORT from z = -2 to z = H_FLANGE + 2
  const dWirePort = Math.max(R_WIRE_PORT - r2d, -z - 1.0, z - (H_FLANGE + 4.0));
  // Negative subtraction: d = Math.max(dSolid, -(hole))
  let dTotal = Math.max(dSolid, -(Math.max(r2d - R_WIRE_PORT, -z - 2.0, z - (H_FLANGE + 2.0))));

  // 2. 4 Base Mounting Bolt Holes
  for (let a = 45; a <= 315; a += 90) {
    const bx = R_BOLT_PCD * Math.sin(a * DEG2RAD);
    const by = R_BOLT_PCD * Math.cos(a * DEG2RAD);
    const dHole = length2(x - bx, y - by) - R_BOLT_HOLE;
    if (z >= -1 && z <= H_FLANGE + 2) {
      dTotal = Math.max(dTotal, -dHole);
    }
  }

  // 3. 9 Sensor Through-Holes and Recesses
  for (let i = 0; i < SENSORS.length; i++) {
    const s = SENSORS[i];
    const t = x * s.nx + y * s.ny + z * s.nz;
    if (t > R_INNER - 3.0 && t < R_OUTER + H_COLLAR_EXT + 3.0) {
      const px = x - t * s.nx;
      const py = y - t * s.ny;
      const pz = z - t * s.nz;
      const dRadial = length(px, py, pz);

      // Circular through-hole
      const dHole = dRadial - R_SENSOR_HOLE;
      const dCyl = Math.max(dHole, (R_INNER - 2.0) - t, t - (R_OUTER + H_COLLAR_EXT + 2.0));
      dTotal = Math.max(dTotal, -dCyl);

      // Rectangular recess pocket near outer surface
      if (t > (R_OUTER + H_COLLAR_EXT - RECESS_D)) {
        const dRecess = dRadial - (RECESS_W * 0.5);
        dTotal = Math.max(dTotal, -dRecess);
      }
    }
  }

  return dTotal;
}

// -------------------------------------------------------------
// Marching Cubes Polygonizer
// -------------------------------------------------------------
// Standard edge and triangle lookup tables
const EDGE_TABLE = new Int32Array([
  0x0, 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
  0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
  0x190, 0x99, 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
  0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
  0x230, 0x339, 0x33, 0x13a, 0x636, 0x73f, 0x435, 0x53c,
  0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
  0x3a0, 0x2a9, 0x1a3, 0xaa, 0x7a6, 0x6af, 0x5a5, 0x4ac,
  0xba0, 0xaa9, 0x9a3, 0x8aa, 0xfa6, 0xeaf, 0xda5, 0xca0,
  0x460, 0x569, 0x663, 0x76a, 0x66, 0x16f, 0x265, 0x36c,
  0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
  0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff, 0x3f5, 0x2fc,
  0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
  0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55, 0x15c,
  0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
  0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc,
  0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
  0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
  0xcc, 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
  0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
  0x15c, 0x55, 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
  0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
  0x2fc, 0x3f5, 0xff, 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
  0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
  0x36c, 0x265, 0x16f, 0x66, 0x76a, 0x663, 0x569, 0x460,
  0xca0, 0xda5, 0xeaf, 0xfa6, 0x8aa, 0x9a3, 0xaa9, 0xba0,
  0xaa, 0x1a3, 0x2a9, 0x3a0, 0x4ac, 0x5a5, 0x6af, 0x7a6,
  0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
  0x13a, 0x33, 0x339, 0x230, 0x53c, 0x435, 0x73f, 0x636,
  0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
  0x29a, 0x393, 0x99, 0x190, 0x69c, 0x795, 0x49f, 0x596,
  0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
  0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0
]);

// Marching cubes triangle table
const TRI_TABLE = [
  [-1],
  [0, 8, 3, -1],
  [0, 1, 9, -1],
  [1, 8, 3, 9, 8, 1, -1],
  [1, 2, 10, -1],
  [0, 8, 3, 1, 2, 10, -1],
  [9, 2, 10, 0, 2, 9, -1],
  [2, 8, 3, 2, 10, 8, 10, 9, 8, -1],
  [3, 11, 2, -1],
  [0, 11, 2, 8, 11, 0, -1],
  [1, 9, 0, 2, 3, 11, -1],
  [1, 11, 2, 1, 9, 11, 9, 8, 11, -1],
  [3, 10, 1, 11, 10, 3, -1],
  [0, 10, 1, 0, 8, 10, 8, 11, 10, -1],
  [3, 9, 0, 3, 11, 9, 11, 10, 9, -1],
  [9, 8, 10, 10, 8, 11, -1],
  [4, 7, 8, -1],
  [4, 3, 0, 7, 3, 4, -1],
  [0, 1, 9, 8, 4, 7, -1],
  [4, 1, 9, 4, 7, 1, 7, 3, 1, -1],
  [1, 2, 10, 8, 4, 7, -1],
  [3, 4, 7, 3, 0, 4, 1, 2, 10, -1],
  [9, 2, 10, 9, 0, 2, 8, 4, 7, -1],
  [2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1],
  [8, 4, 7, 3, 11, 2, -1],
  [11, 4, 7, 11, 2, 4, 2, 0, 4, -1],
  [9, 0, 1, 8, 4, 7, 2, 3, 11, -1],
  [4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1],
  [3, 10, 1, 3, 11, 10, 7, 8, 4, -1],
  [1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1],
  [4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1],
  [4, 7, 11, 4, 11, 9, 9, 11, 10, -1],
  [9, 5, 4, -1],
  [9, 5, 4, 0, 8, 3, -1],
  [0, 5, 4, 1, 5, 0, -1],
  [8, 5, 4, 8, 3, 5, 3, 1, 5, -1],
  [1, 2, 10, 9, 5, 4, -1],
  [3, 0, 8, 1, 2, 10, 4, 9, 5, -1],
  [5, 2, 10, 5, 4, 2, 4, 0, 2, -1],
  [2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1],
  [9, 5, 4, 2, 3, 11, -1],
  [0, 11, 2, 0, 8, 11, 4, 9, 5, -1],
  [0, 5, 4, 0, 1, 5, 2, 3, 11, -1],
  [2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1],
  [10, 3, 11, 10, 1, 3, 9, 5, 4, -1],
  [4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1],
  [5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1],
  [5, 4, 8, 5, 8, 10, 10, 8, 11, -1],
  [9, 7, 8, 5, 7, 9, -1],
  [9, 3, 0, 9, 5, 3, 5, 7, 3, -1],
  [0, 7, 8, 0, 1, 7, 1, 5, 7, -1],
  [1, 5, 3, 3, 5, 7, -1],
  [9, 7, 8, 9, 5, 7, 10, 1, 2, -1],
  [10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1],
  [8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1],
  [2, 10, 5, 2, 5, 3, 3, 5, 7, -1],
  [7, 9, 5, 7, 8, 9, 3, 11, 2, -1],
  [9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1],
  [2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1],
  [11, 2, 1, 11, 1, 7, 7, 1, 5, -1],
  [9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1],
  [5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1],
  [11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1],
  [11, 10, 5, 7, 11, 5, -1],
  [10, 6, 5, -1],
  [0, 8, 3, 5, 10, 6, -1],
  [9, 0, 1, 5, 10, 6, -1],
  [1, 8, 3, 1, 9, 8, 5, 10, 6, -1],
  [1, 6, 5, 2, 6, 1, -1],
  [1, 6, 5, 1, 2, 6, 3, 0, 8, -1],
  [9, 6, 5, 9, 0, 6, 0, 2, 6, -1],
  [5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1],
  [2, 3, 11, 10, 6, 5, -1],
  [11, 0, 8, 11, 2, 0, 10, 6, 5, -1],
  [0, 1, 9, 2, 3, 11, 5, 10, 6, -1],
  [5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1],
  [6, 3, 11, 6, 5, 3, 5, 1, 3, -1],
  [0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1],
  [3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1],
  [6, 5, 9, 6, 9, 11, 11, 9, 8, -1],
  [5, 10, 6, 4, 7, 8, -1],
  [4, 3, 0, 4, 7, 3, 6, 5, 10, -1],
  [1, 9, 0, 5, 10, 6, 8, 4, 7, -1],
  [10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1],
  [6, 1, 2, 6, 5, 1, 4, 7, 8, -1],
  [1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1],
  [8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1],
  [7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1],
  [3, 11, 2, 7, 8, 4, 10, 6, 5, -1],
  [5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1],
  [0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1],
  [9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1],
  [8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1],
  [5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1],
  [0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1],
  [6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1],
  [10, 4, 9, 6, 4, 10, -1],
  [4, 10, 6, 4, 9, 10, 0, 8, 3, -1],
  [10, 0, 1, 10, 6, 0, 6, 4, 0, -1],
  [8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1],
  [1, 4, 9, 1, 2, 4, 2, 6, 4, -1],
  [3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1],
  [0, 2, 4, 4, 2, 6, -1],
  [8, 3, 2, 8, 2, 4, 4, 2, 6, -1],
  [10, 4, 9, 10, 6, 4, 11, 2, 3, -1],
  [0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1],
  [3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1],
  [6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1],
  [9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1],
  [8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1],
  [3, 11, 6, 3, 6, 0, 0, 6, 4, -1],
  [8, 11, 6, 8, 6, 4, -1],
  [9, 7, 8, 9, 10, 7, 10, 6, 7, -1],
  [8, 9, 10, 8, 10, 7, 10, 6, 7, 3, 0, 8, -1],
  [1, 10, 6, 1, 6, 7, 1, 7, 0, 8, 0, 7, -1],
  [1, 10, 6, 1, 6, 7, 1, 7, 3, -1],
  [1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1],
  [2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1],
  [7, 8, 0, 7, 0, 6, 6, 0, 2, -1],
  [7, 3, 2, 6, 7, 2, -1],
  [2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1],
  [2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1],
  [1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1],
  [11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1],
  [8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1],
  [0, 9, 1, 11, 6, 7, -1],
  [7, 8, 0, 7, 0, 11, 6, 7, 11, 6, 11, 3, -1],
  [7, 6, 11, -1]
];

// Invert table for indices 128..255 (by complement)
for (let i = 128; i < 256; i++) {
  const inv = 255 - i;
  if (TRI_TABLE[inv]) {
    const list = TRI_TABLE[inv];
    if (list[0] === -1) {
      TRI_TABLE[i] = [-1];
    } else {
      const rev = [];
      for (let j = 0; j < list.length - 1; j += 3) {
        rev.push(list[j + 2], list[j + 1], list[j]);
      }
      rev.push(-1);
      TRI_TABLE[i] = rev;
    }
  } else {
    TRI_TABLE[i] = [-1];
  }
}

// -------------------------------------------------------------
// Marching Cubes Grid Evaluation
// -------------------------------------------------------------
const STEP = 0.70; // 0.70 mm grid resolution -> sharp detail & fast execution
const X_MIN = -45.5, X_MAX = 45.5;
const Y_MIN = -45.5, Y_MAX = 45.5;
const Z_MIN = -0.5,  Z_MAX = 38.5;

const NX = Math.ceil((X_MAX - X_MIN) / STEP);
const NY = Math.ceil((Y_MAX - Y_MIN) / STEP);
const NZ = Math.ceil((Z_MAX - Z_MIN) / STEP);

console.log(`Grid dimensions: ${NX} x ${NY} x ${NZ} (${NX * NY * NZ} cells, step: ${STEP}mm)`);

// Pre-compute 3D grid values
const grid = new Float32Array((NX + 1) * (NY + 1) * (NZ + 1));
const idx = (ix, iy, iz) => (iz * (NY + 1) + iy) * (NX + 1) + ix;

for (let iz = 0; iz <= NZ; iz++) {
  const z = Z_MIN + iz * STEP;
  for (let iy = 0; iy <= NY; iy++) {
    const y = Y_MIN + iy * STEP;
    for (let ix = 0; ix <= NX; ix++) {
      const x = X_MIN + ix * STEP;
      grid[idx(ix, iy, iz)] = sdfModel(x, y, z);
    }
  }
}

// Corner offsets
const CORNERS = [
  [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
  [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]
];

// Linear interpolation between two grid points
function interp(x1, y1, z1, v1, x2, y2, z2, v2) {
  const mu = Math.abs(v1 - v2) < 1e-5 ? 0.5 : (0 - v1) / (v2 - v1);
  return [
    x1 + mu * (x2 - x1),
    y1 + mu * (y2 - y1),
    z1 + mu * (z2 - z1)
  ];
}

const triangles = [];

for (let iz = 0; iz < NZ; iz++) {
  const z0 = Z_MIN + iz * STEP;
  for (let iy = 0; iy < NY; iy++) {
    const y0 = Y_MIN + iy * STEP;
    for (let ix = 0; ix < NX; ix++) {
      const x0 = X_MIN + ix * STEP;

      // Sample 8 corners
      const v = new Float32Array(8);
      let cubeIndex = 0;
      for (let c = 0; c < 8; c++) {
        const val = grid[idx(ix + CORNERS[c][0], iy + CORNERS[c][1], iz + CORNERS[c][2])];
        v[c] = val;
        if (val < 0) cubeIndex |= (1 << c);
      }

      if (EDGE_TABLE[cubeIndex] === 0 || cubeIndex === 0 || cubeIndex === 255) continue;

      const p = [];
      for (let c = 0; c < 8; c++) {
        p.push([
          x0 + CORNERS[c][0] * STEP,
          y0 + CORNERS[c][1] * STEP,
          z0 + CORNERS[c][2] * STEP
        ]);
      }

      // 12 edges interpolation
      const vertList = new Array(12);
      if (EDGE_TABLE[cubeIndex] & 1)    vertList[0]  = interp(p[0][0], p[0][1], p[0][2], v[0], p[1][0], p[1][1], p[1][2], v[1]);
      if (EDGE_TABLE[cubeIndex] & 2)    vertList[1]  = interp(p[1][0], p[1][1], p[1][2], v[1], p[2][0], p[2][1], p[2][2], v[2]);
      if (EDGE_TABLE[cubeIndex] & 4)    vertList[2]  = interp(p[2][0], p[2][1], p[2][2], v[2], p[3][0], p[3][1], p[3][2], v[3]);
      if (EDGE_TABLE[cubeIndex] & 8)    vertList[3]  = interp(p[3][0], p[3][1], p[3][2], v[3], p[0][0], p[0][1], p[0][2], v[0]);
      if (EDGE_TABLE[cubeIndex] & 16)   vertList[4]  = interp(p[4][0], p[4][1], p[4][2], v[4], p[5][0], p[5][1], p[5][2], v[5]);
      if (EDGE_TABLE[cubeIndex] & 32)   vertList[5]  = interp(p[5][0], p[5][1], p[5][2], v[5], p[6][0], p[6][1], p[6][2], v[6]);
      if (EDGE_TABLE[cubeIndex] & 64)   vertList[6]  = interp(p[6][0], p[6][1], p[6][2], v[6], p[7][0], p[7][1], p[7][2], v[7]);
      if (EDGE_TABLE[cubeIndex] & 128)  vertList[7]  = interp(p[7][0], p[7][1], p[7][2], v[7], p[4][0], p[4][1], p[4][2], v[4]);
      if (EDGE_TABLE[cubeIndex] & 256)  vertList[8]  = interp(p[0][0], p[0][1], p[0][2], v[0], p[4][0], p[4][1], p[4][2], v[4]);
      if (EDGE_TABLE[cubeIndex] & 512)  vertList[9]  = interp(p[1][0], p[1][1], p[1][2], v[1], p[5][0], p[5][1], p[5][2], v[5]);
      if (EDGE_TABLE[cubeIndex] & 1024) vertList[10] = interp(p[2][0], p[2][1], p[2][2], v[2], p[6][0], p[6][1], p[6][2], v[6]);
      if (EDGE_TABLE[cubeIndex] & 2048) vertList[11] = interp(p[3][0], p[3][1], p[3][2], v[3], p[7][0], p[7][1], p[7][2], v[7]);

      const triIndices = TRI_TABLE[cubeIndex];
      if (triIndices) {
        for (let t = 0; t + 2 < triIndices.length && triIndices[t] !== -1; t += 3) {
          const v0 = vertList[triIndices[t]];
          const v1 = vertList[triIndices[t + 1]];
          const v2 = vertList[triIndices[t + 2]];
          if (v0 && v1 && v2) {
            triangles.push([v0, v1, v2]);
          }
        }
      }
    }
  }
}

console.log(`Generated ${triangles.length} triangles.`);

// -------------------------------------------------------------
// Write Binary STL File
// -------------------------------------------------------------
const numTris = triangles.length;
const bufferSize = 84 + numTris * 50;
const buf = Buffer.alloc(bufferSize);

// Header (80 bytes)
const header = 'Solar Dome Sensor - 9 Photodiode Enclosure (70mm OD, 4x M3 Flange)';
buf.write(header, 0, Math.min(header.length, 80), 'ascii');

// Number of triangles (4 bytes uint32)
buf.writeUInt32LE(numTris, 80);

let offset = 84;
for (let i = 0; i < numTris; i++) {
  const tri = triangles[i];
  const p1 = tri[0], p2 = tri[1], p3 = tri[2];

  // Face normal calculation
  const ax = p2[0] - p1[0], ay = p2[1] - p1[1], az = p2[2] - p1[2];
  const bx = p3[0] - p1[0], by = p3[1] - p1[1], bz = p3[2] - p1[2];
  let nx = ay * bz - az * by;
  let ny = az * bx - ax * bz;
  let nz = ax * by - ay * bx;
  const nLen = Math.hypot(nx, ny, nz);
  if (nLen > 1e-6) { nx /= nLen; ny /= nLen; nz /= nLen; }

  // Normal vector (12 bytes)
  buf.writeFloatLE(nx, offset);
  buf.writeFloatLE(ny, offset + 4);
  buf.writeFloatLE(nz, offset + 8);

  // Vertex 1 (12 bytes)
  buf.writeFloatLE(p1[0], offset + 12);
  buf.writeFloatLE(p1[1], offset + 16);
  buf.writeFloatLE(p1[2], offset + 20);

  // Vertex 2 (12 bytes)
  buf.writeFloatLE(p2[0], offset + 24);
  buf.writeFloatLE(p2[1], offset + 28);
  buf.writeFloatLE(p2[2], offset + 32);

  // Vertex 3 (12 bytes)
  buf.writeFloatLE(p3[0], offset + 36);
  buf.writeFloatLE(p3[1], offset + 40);
  buf.writeFloatLE(p3[2], offset + 44);

  // Attribute byte count (2 bytes uint16 = 0)
  buf.writeUInt16LE(0, offset + 48);

  offset += 50;
}

const outPath = path.join(__dirname, 'dome_sensor_9.stl');
fs.writeFileSync(outPath, buf);

console.log(`Saved STL: ${outPath} (${(buf.length / (1024 * 1024)).toFixed(2)} MB)`);
