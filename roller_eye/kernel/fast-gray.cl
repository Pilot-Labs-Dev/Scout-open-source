// Copyright (C) 2021  Majid Geravand
// Copyright (C) 2021  Gfuse

// Enable OpenCL 32-bit integer atomic functions.
#pragma OPENCL EXTENSION cl_khr_global_int32_base_atomics : enable

__kernel void fast_gray(
    __read_only  image2d_t   image,
    __global     int2      * corners,
    __global     int       * icorner,
               int       fast_count
) {
    
    // Prepare a suitable OpenCL image sampler.
    sampler_t const sampler = CLK_ADDRESS_CLAMP | CLK_FILTER_NEAREST;

    // Use global work item as 2D image coordinates.
    int  const x   = get_global_id(0);
    int  const y   = get_global_id(1);
    if(x>5 && x<get_image_width(image)-5 && y<get_image_height(image)-5 && y>5){
        int2 const xy  = (int2)(x, y);
        // Read the candidate pixel.
        int  const p00 = read_imageui(image, sampler, xy).x;

        // Read other pixels in a circle around the candidate pixel.
        int  const p01 = read_imageui(image, sampler, xy + (int2)( 0,  3)).x;
        int  const p05 = read_imageui(image, sampler, xy + (int2)( 3,  0)).x;
        int  const p09 = read_imageui(image, sampler, xy + (int2)( 0, -3)).x;
        int  const p13 = read_imageui(image, sampler, xy + (int2)(-3,  0)).x;

        // Check the absolute difference of each circle pixel.
        int  const d01 = (abs(p01 - p00) > FAST_THRESH);
        int  const d05 = (abs(p05 - p00) > FAST_THRESH);
        int  const d09 = (abs(p09 - p00) > FAST_THRESH);
        int  const d13 = (abs(p13 - p00) > FAST_THRESH);

        // Check if any two adjacent circle pixels have a high absolute difference.

        if ((d01 && d05) ||(d05 && d09) ||(d09 && d13) ||(d13 && d01)) {
                // Read other pixels in a circle around the candidate pixel.
            int  const p02 = read_imageui(image, sampler, xy + (int2)( 1,  3)).x;
            int  const p03 = read_imageui(image, sampler, xy + (int2)( 2,  2)).x;
            int  const p04 = read_imageui(image, sampler, xy + (int2)( 3,  1)).x;
            int  const p06 = read_imageui(image, sampler, xy + (int2)( 3, -1)).x;
            int  const p07 = read_imageui(image, sampler, xy + (int2)( 2, -2)).x;
            int  const p08 = read_imageui(image, sampler, xy + (int2)( 1, -3)).x;
            int  const p10 = read_imageui(image, sampler, xy + (int2)(-1, -3)).x;
            int  const p11 = read_imageui(image, sampler, xy + (int2)(-2, -2)).x;
            int  const p12 = read_imageui(image, sampler, xy + (int2)(-3, -1)).x;
            int  const p14 = read_imageui(image, sampler, xy + (int2)(-3,  1)).x;
            int  const p15 = read_imageui(image, sampler, xy + (int2)(-2,  2)).x;
            int  const p16 = read_imageui(image, sampler, xy + (int2)(-1,  3)).x;
            // Select the maximum score.
            int     sco = p00;
                    sco = max(sco, p01);
                    sco = max(sco, p02);
                    sco = max(sco, p03);
                    sco = max(sco, p04);
                    sco = max(sco, p05);
                    sco = max(sco, p06);
                    sco = max(sco, p07);
                    sco = max(sco, p08);
                    sco = max(sco, p09);
                    sco = max(sco, p10);
                    sco = max(sco, p11);
                    sco = max(sco, p12);
                    sco = max(sco, p13);
                    sco = max(sco, p14);
                    sco = max(sco, p15);

            // Keep this score if it is as good as the maximum.
            if (p00 >= sco) {
                // Atomically append to corner buffer.
                int const icorn = atom_inc(icorner);
                if (icorn < fast_count)
                    corners[icorn] = xy;
            }
        }

    }
    
}

