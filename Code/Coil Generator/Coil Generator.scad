// 3D Printed Electromagentic Coil Generator
// Brian Minnick 2024

$fn=25;

// Coil Parameters
num_coil_layers = 12;
num_coil_rings = 2;
inner_diameter = 5;

// Electrical Parameters
current = 5; //Amps
num_turns = num_coil_layers*num_coil_rings;
mu = 0.0000012566370614359172;

// Printer Parameters
layer_width = 0.4;
layer_height = 0.2;

// Trace Parameters (Number of Layers)
trace_layers_thickness = 2;
trace_layers_depth = 2;

// Insulation Parameters (Number of Layers)
ins_layers_radial = 1;
ins_layers_axial = 1;

// Trace Parameters
trace_width = layer_width*trace_layers_thickness;
trace_depth = layer_height*trace_layers_depth;

// Insulation Parameters
radial_insulation = layer_width*ins_layers_radial;
layer_insulation = layer_height*ins_layers_axial;

// Calculate Magnetic Force
coil_length = (num_coil_layers*layer_height*2)/1000;
A = 3.141592*(inner_diameter/2000)^2;
F = ((num_turns*current)^2*mu*A)/(2*0.0005^2);

echo(A);
echo(F, "N of force");


// Call Function
generate_coil_stack(
    num_coil_layers,
    num_coil_rings,
    inner_diameter,
    trace_width,
    trace_depth,
    radial_insulation,
    layer_insulation
);

// Generate insulation

//difference() {
//    cylinder(h=num_coil_layers*(trace_depth+layer_insulation), r=inner_diameter/2+(num_coil_rings+1)*(trace_width+radial_insulation)+radial_insulation);
//    
//    generate_coil_stack(
//        num_coil_layers,
//        num_coil_rings,
//        inner_diameter,
//        trace_width,
//        trace_depth,
//        radial_insulation,
//        layer_insulation
//    );
//}


// Generate a whole coil
module generate_coil_stack(
    n_coil_layers,
    n_coil_rings,
    id,
    t_width,
    t_depth,
    i_radial,
    i_axial
){
    union() {
        for (r = [0 : num_coil_layers - 1]) {
            if (r%2==1){
                union(){
                    // form the coil layer
                    translate([0,0,r*(t_depth+i_axial)]) generate_coil_layer(n_coil_rings,id,t_width,t_depth,i_radial,i_axial);
                    
                    // form the connection between this layer and the next
                    translate([0,-1*((id/2)+t_width/2+i_radial+n_coil_rings*(t_width+i_radial)),(2*t_depth+i_axial)/2+(r*(t_depth+i_axial))])
                    
                    cube([t_depth*2, t_width, 2*t_depth+i_axial], center=true);
                }
            }
            if (r%2==0){
                union() {
                    // form the coil layer and mirror it because we are on an odd coil number
                    mirror ([1,0,0]) 
                    translate([0,0,r*(t_depth+i_axial)]) 
                    generate_coil_layer(n_coil_rings,id,t_width,t_depth,i_radial,i_axial);
                    
                    // form the connection between this layer and the next
                    translate([0,-1*((id/2)+t_width/2+i_radial),(2*t_depth+i_axial)/2+(r*(t_depth+i_axial))])
                    
                    cube([t_depth*2, t_width, 2*t_depth+i_axial], center=true);
                }
            }
        }
    }
    // With the coil stack constructed, we want to make the insulation
}

// Generate a single coil layer
module generate_coil_layer(
n_coil_rings,
id,
t_width,
t_depth,
i_radial,
i_axial){
    coil_layer_depth = i_axial+t_depth;
    center_offset = (t_width+i_radial)/2;
    //outer_radius = (id/2)+center_offset+t_width*n_coil_rings+i_axial*(n_coil_rings+1);
    
    outer_radius = (id/2+(n_coil_rings+1)*(t_width+i_radial)+i_radial);
    
    r_inc = t_width+i_radial;
    
    //difference() {
        //cylinder(h=coil_layer_depth, r=(id/2)+(i_radial));
        //cylinder(h=coil_layer_depth, r=(id/2)+(i_radial));
    //}
    
    union(){
        for ( i = [0 : 1 : n_coil_rings - 1]){
            gen_half(0, (id/2)+i_radial+(i*r_inc), t_width, t_depth, outer_radius);
            rotate(180) gen_half(center_offset, (id/2)+i_radial+center_offset+(i*r_inc), t_width, t_depth, outer_radius);
        }
    }
    
}

module gen_half(
coil_center,
ir,
t_width,
t_depth,
outer_radius,
){
    translate([0,coil_center,0])
    difference(){
        cylinder(h=t_depth, r=ir+t_width);
        translate([0, 0, -0.01]) cylinder(h=t_depth+0.02, r=ir);
        translate([outer_radius,0,0]) cube(outer_radius*2, center = true);
    };
    
}