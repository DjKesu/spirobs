// SpiRob Spiral Robot in OpenSCAD
// Based on the logarithmic spiral design with hexagonal segments

/* [Basic Parameters] */
// Taper angle in degrees (5, 10, or 15 recommended)
taper_angle = 10;

// Number of discrete units in the spiral
unit_count = 4;

// Total length of the spiral robot (mm)
total_length = 30;

// assume every unit is 18.75 mm

// Width at tip (mm)
tip_width = 5.5;

// Gap between units as percentage of unit height
gap_percent = 20;

// Cable hole diameter for #18 mason twine (1.52 mm)
cable_diameter = 1.52;

// Number of cables (2)
cable_count = 2;

/* [Advanced Parameters] */
// Discretization angle (degrees)
discretization_angle = 30;

// Calculate b parameter based on taper angle
function calc_b_param(angle) = 
    // This is an approximation based on the paper's values
    (angle == 5) ? 0.07 :
    (angle == 10) ? 0.15 :
    (angle == 15) ? 0.22 : 0.15;

// The b parameter of the logarithmic spiral
b_param = calc_b_param(taper_angle);

/* [Hidden] */
// Calculate the scaling ratio between units
beta = exp(b_param * discretization_angle * PI / 180);
// Height of each unit
unit_height = total_length / unit_count;
// Gap between units
gap_height = unit_height * gap_percent / 100;
// Actual height of each solid unit
solid_height = unit_height - gap_height;

// Function to create a single unit with cable holes
module create_unit(scale_factor, position_z) {
    scaled_width = tip_width * scale_factor;
    
    difference() {
        // The main hexagonal prism unit
        translate([0, 0, position_z])
        linear_extrude(height = solid_height)
        hexagon(scaled_width);
        
        // Cable holes
        if (cable_count >= 2) {
            // Calculate positions for cable holes based on scaled width
            offset_x = scaled_width/5;
            offset_y = scaled_width/5;
            
            // First cable hole
            translate([offset_x, offset_y/5, -1])
            cylinder(h = total_length + 2, d = cable_diameter, $fn = 16);
            
            // Second cable hole
            translate([-offset_x, offset_y/5, -1])
            cylinder(h = total_length, d = cable_diameter, $fn = 20);
            
        }
    }
}

// Function to create a hexagon
module hexagon(width) {
    radius = width / 2;
    angles = [0, 60, 120, 180, 240, 300];
    points = [for (a = angles) [radius * cos(a), radius * sin(a)]];
    polygon(points);
}

// Create elastic layer along the center (optional)
module elastic_layer() {
    central_axis_diameter = tip_width * 0.2;
    
    translate([0, 0, 0])
    cylinder(h = total_length, d = central_axis_diameter, $fn = 16);
}

// Main module to create the complete SpiRob
module spirob() {
    // Create the units
    for (i = [0:unit_count-1]) {
        scale_factor = pow(beta, i);
        position_z = i * unit_height;
        create_unit(scale_factor, position_z);
    }
    
    // Add elastic layer (uncomment if needed)
    // elastic_layer();
}

// Create the SpiRob
spirob();
