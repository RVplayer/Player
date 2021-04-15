function is_onGround = on_ground(z, frame_height)
    if z-frame_height < 0.001
       is_onGround = 1;
       return;
    else
        is_onGround = 0;
        return;
    end
end