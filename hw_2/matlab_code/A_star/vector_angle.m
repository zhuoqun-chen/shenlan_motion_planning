function val = vector_angle(vec1_x, vec1_y, vec2_x, vec2_y)
    if ( vec1_x*vec2_x+vec1_y*vec2_y == 0 || vec1_x*vec2_x+vec1_y*vec2_y == 1 )
        val = 1;
    else
        val = -1;
end