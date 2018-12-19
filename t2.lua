-- This script is used for realtime emulation of the environment in V-REP

function sysCall_init()

    -- Add required handles here


    c_tree_pos_handle = sim.getObjectHandle('Position_hoop3')
    c_tree_ori_handle = sim.getObjectHandle('Orientation_hoop3')


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
    -- id 2 is for cashew tree
    pose = msg.markers[1].pose.pose.orientation
    sim.setObjectQuaternion(c_tree_ori_handle,-1,{pose.x,pose.y,-pose.z,-pose.w})
	


end

function whycon_callback(msg)
    -- Get the position of the whycon marker and set the position of the food tree and non-food tree using Position_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectPosition
    pos = msg.poses[1].position
    sim.setObjectPosition(c_tree_pos_handle,-1,{pos.x/4,-pos.y/4,0.5}) 
end

function key_callback(msg)
    -- Read key input to set or unset position and orientation of food and non-food trees
    if msg.data == 70 then
        -- set orientation of current tree
    elseif msg.data == 60 then
        -- set position of current tree
    elseif msg.data == 0 then
        -- reset current tree orientation to live mode
    elseif msg.data == 10 then
        -- reset current tree  position to live mode
    elseif msg.data == 20 then
        -- move to next tree
    else 
    	print("not a command for this script")
    end
    



end
