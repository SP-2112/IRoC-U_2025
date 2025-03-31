function takeoff_hover_land()
	local target_altitude = 1.5 -- in metres
	local hover_time = 1000
	if not arming:is_armed() then
        vehicle:set_mode(20)
		arming:arm()
		gcs:send_text(0, "Arming vehicle ... ")
		return takeoff_hover_land, 2000 --Rerun after 2 seconds
	end
	
	if vehicle:get_mode() ~= 20 then
		vehicle:set_mode(20)
        local check_takeoff=vehicle:start_takeoff(target_altitude)
		gcs:send_text(0, "Switching to Guided mode")
		return takeoff_hover_land, 2000
	end
	
	-- if ahrs:get_hagl()/1000 < target_altitude - 0.2 then
	-- 	vehicle:start_takeoff(target_altitude)
	-- 	gcs:send_text(0, "Taking off ".. target_altitude.. "meters")
	-- 	return takeoff_hover_land, 1000
	-- end
    if not vehicle:start_takeoff(target_altitude) then
        gcs:send_text(0,"Unable to takeoff")
    end
	gcs:send_text(0, "Hovering ")
	return takeoff_hover_land, hover_time
end

function land()
	gcs:send_text(0, "Initiating landing")
	vehicle:set_mode(9)
	return nil
	
end

function trig_check()
	local trig_mode = 5
	if vehicle:get_mode() ~= trig_mode then
		return trig_check, 1000
	end
	gcs:send_text(0, "triggering autonomous mode..")
	return takeoff_hover_land, 100
end
return trig_check()