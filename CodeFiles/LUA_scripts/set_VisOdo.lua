-- Lua script to monitor flight mode and update VISO_TYPE parameter accordingly

function update_viso_type()
    local mode = vehicle:get_mode() -- Get current flight mode
    
    if mode == 0 then  -- 0 corresponds to STABILIZE mode in ArduPilot 
        param:set("VISO_TYPE", 0) -- Disable Visual Odometry
    else
        param:set("VISO_TYPE", 1) -- Enable Visual Odometry
    end
end

function loop()
    update_viso_type()
    return loop, 100
end

return loop()