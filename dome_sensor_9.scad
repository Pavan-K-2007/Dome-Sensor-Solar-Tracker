// =============================================================================
// LIGHT-ADAPTIVE SOLAR TRACKER - 9-SENSOR DOME
// 3D Printable Hollow Hemispherical Sensor Enclosure
// Compatible with OpenSCAD (https://openscad.org) and FreeCAD
// =============================================================================

// Mesh resolution for high-quality export (F6 to Render, then File -> Export as STL)
$fn = 120;

// =============================================================================
// PARAMETRIC DIMENSIONS (All values in millimeters)
// =============================================================================

// Dome Sphere Dimensions
dome_outer_radius  = 35.0; // 70 mm outer diameter
dome_wall_thick    = 3.2;  // Wall thickness (optimized for 0.2mm layer FDM prints)
dome_inner_radius  = dome_outer_radius - dome_wall_thick; // 31.8 mm inner cavity

// Base Mounting Flange
flange_outer_rad   = 44.0; // 88 mm base diameter
flange_height      = 3.5;  // 3.5 mm base flange thickness
flange_bolt_pcd    = 39.0; // Bolt circle radius (78 mm PCD)
flange_bolt_dia    = 3.4;  // Clearance hole for M3 mounting bolts
flange_bolt_csk    = 6.2;  // M3 bolt head countersink diameter
wire_port_radius   = 14.0; // 28 mm bottom central cable routing hole

// Sensor Port Dimensions (Designed for BPW34 PIN Photodiode / 5mm Sensors)
sensor_aperture_rad  = 3.2; // 6.4 mm light entry & lead through-hole
sensor_collar_rad    = 6.8; // 13.6 mm outer reinforcing collar around each sensor
sensor_collar_height = 1.6; // Slight protrusion to ensure flat, perpendicular mounting face
sensor_recess_w      = 5.6; // BPW34 body width (5.4 mm + 0.2 mm clearance)
sensor_recess_l      = 4.6; // BPW34 body length (4.4 mm + 0.2 mm clearance)
sensor_recess_depth  = 1.8; // Recess seat depth for flush seating

// =============================================================================
// 9 SENSOR ORIENTATIONS: [Azimuth Angle (°), Elevation Angle (°)]
// - Azimuth: 0° = North, 90° = East, 180° = South, 270° = West
// - Elevation: 90° = Zenith (straight up), 0° = Horizon
// =============================================================================
sensor_positions = [
  [   0, 90 ], // S1: Zenith / Top
  [   0, 60 ], // S2: North (Upper Ring)
  [  90, 60 ], // S3: East  (Upper Ring)
  [ 180, 60 ], // S4: South (Upper Ring)
  [ 270, 60 ], // S5: West  (Upper Ring)
  [  45, 30 ], // S6: North-East (Lower Ring - Low sun / Horizon tracking)
  [ 135, 30 ], // S7: South-East (Lower Ring - Morning sunrise)
  [ 225, 30 ], // S8: South-West (Lower Ring - Evening sunset)
  [ 315, 30 ]  // S9: North-West (Lower Ring - Low sun / Horizon tracking)
];

// =============================================================================
// MAIN ASSEMBLY
// =============================================================================

module dome_sensor_assembly() {
  difference() {
    // --- 1. POSITIVE BODIES (Dome + Flange + Collars) ---
    union() {
      // Outer Hemispherical Shell
      difference() {
        sphere(r = dome_outer_radius);
        // Trim bottom below z = 0
        translate([0, 0, -dome_outer_radius])
          cube([dome_outer_radius * 2.5, dome_outer_radius * 2.5, dome_outer_radius * 2], center = true);
      }

      // Base Mounting Flange
      translate([0, 0, 0])
        cylinder(r = flange_outer_rad, h = flange_height);

      // Reinforcing Collars on the outer dome for each sensor
      for (pos = sensor_positions) {
        rotate([0, 0, pos[0]])
          rotate([0, 90 - pos[1], 0])
            translate([0, 0, dome_outer_radius - 1.0])
              cylinder(r = sensor_collar_rad, h = sensor_collar_height + 1.0);
      }
    }

    // --- 2. NEGATIVE BODIES (Hollow Cavity + Through-holes + Mounting Holes) ---
    
    // Inner Hollow Cavity
    sphere(r = dome_inner_radius);

    // Central Bottom Wire Passthrough
    translate([0, 0, -1])
      cylinder(r = wire_port_radius, h = flange_height + 4);

    // Base Flange M3 Mounting Bolt Holes (4x at 90° spacing)
    for (angle = [45, 135, 225, 315]) {
      rotate([0, 0, angle]) {
        translate([flange_bolt_pcd, 0, -1]) {
          cylinder(r = flange_bolt_dia / 2, h = flange_height + 4);
          // Countersink recess for bolt heads
          translate([0, 0, flange_height - 1.2])
            cylinder(r = flange_bolt_csk / 2, h = 4);
        }
      }
    }

    // Sensor Through-Holes & Mounting Recesses
    for (pos = sensor_positions) {
      rotate([0, 0, pos[0]]) {
        rotate([0, 90 - pos[1], 0]) {
          // Circular through-aperture passing into the inner cavity
          translate([0, 0, dome_inner_radius - 2.0])
            cylinder(r = sensor_aperture_rad, h = (dome_outer_radius - dome_inner_radius) + 6.0);

          // Rectangular recess pocket for BPW34 flush seating
          translate([0, 0, dome_outer_radius + sensor_collar_height - sensor_recess_depth])
            cube([sensor_recess_w, sensor_recess_l, sensor_recess_depth + 2.0], center = true);
        }
      }
    }
  }
}

// Render the assembly
dome_sensor_assembly();
