-- This script is used for realtime emulation of the environment in V-REP
-- this scipts uses one aruco and one whycon marker at time to emulate arena in order
-- cashew tree,sal tree, nonfood,tree
-- press a for setting orientation, s for position, k to set next tree
function sysCall_init()

    -- Add required handles here

    position_handles = {sim.getObjectHandle('Position_hoop3'),sim.getObjectHandle('Position_hoop1'),sim.getObjectHandle('obstacle_1')}
    orientation_handles = {sim.getObjectHandle('Orientation_hoop3'),sim.getObjectHandle('Orientation_hoop1')}
    zvals = {0.5,1.1229,1.3345}
    is_current_orientation_set = false
    is_current_position_set = false
    
    -- Subscribing to the required topics 
    aruco_sub = simROS.subscribe('/aruco_marker_publisher/markers', 'aruco_msgs/MarkerArray', 'aruco_callback')
    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')
end


function sysCall_actuation()

end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end



function aruco_callback(msg)
    -- Get the orientation(quaternion) of the ArUco marker and set the orientation of the hoop using Orientation_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectQuaternion
     
    if is_current_orientation_set == false then
        pose = msg.markers[1].pose.pose.orientation
        sim.setObjectQuaternion(orientation_handles[1],-1,{-pose.x,pose.y,-pose.z,pose.w})
	end
    print(sim.getObjectQuaternion(sim.getObjectHandle('Orientation_hoop1'),-1))
    print({pose.x,pose.y,pose.z,-pose.w})


end

function whycon_callback(msg)
    -- Get the position of the whycon marker and set the position of the food tree and non-food tree using Position_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectPosition
    if is_current_position_set == false then
        pos = msg.poses[1].position
        sim.setObjectPosition(position_handles[1],-1,{-pos.x/4,pos.y/4,zvals[1]}) 
    end
end

function key_callback(msg)
    -- Read key input to set or unset position and orientation of food and non-food trees
    if msg.data == 70 then
        -- set orientation of current tree (a)
        is_current_orientation_set = true
        print('current orientation set')
    elseif msg.data == 60 then
        -- set position of current tree (s)
        is_current_position_set = true
        print('current position set')
    elseif msg.data == 0 then
        -- unset current tree orientation(d) 
        is_current_orientation_set = false
        print('unset current orientation')
    elseif msg.data == 10 then
        -- unset current tree  position (i) 
        is_current_position_set = false
        print('unset current position')
    elseif msg.data == 20 then
        -- move to next tree(k)
        table.remove(position_handles,1)
        table.remove(orientation_handles,1)
        table.remove(zvals,1)
        is_current_position_set = false
        is_current_orientation_set = false
    else
        print('Not valid command for this script')
    end
    



end
