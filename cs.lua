function sysCall_init()
    -- This child script contains computing path and publishing it into the path_planning.py for waypoint navigation.


    -- Declaring required handles
    drone_handle = sim.getObjectHandle('eDroneBase')
    collection_handles= sim.getCollectionHandle('Obstacles')

    -- Assigning obstacles handles
    no_of_obstacles = 6
    obstacles_handles = {}
    for i=1,no_of_obstacles do
        table.insert(obstacles_handles,sim.getObjectHandle('obstacle_'..tostring(i)))
    end

    -----------Add other required handles here----------------
    no_of_goals = 2
    goals_handles = {}
    i_handle = sim.getObjectHandle('initial_waypoint')
    table.insert(goals_handles,i_handle)
    for i=1,no_of_goals do
        table.insert(goals_handles,sim.getObjectHandle('goal_'..tostring(i)))
    end
    table.insert(goals_handles,i_handle)
    target_handle = sim.getObjectHandle('target')
    start_handle = sim.getObjectHandle('Start')
    compute_path_flag = false
    --Hint : Goal handles and other required handles
    ----------------------------------------------------------


    ------------Add the path planning task initial details------------------




    --Hint : Creating task, statespace creation, algorithm setting and setting collision pairs
    -- Carefull about the bounds and the collision pairs you set.
    --------------------------------------------------------------------------
    t=simOMPL.createTask('t')
    ss={simOMPL.createStateSpace('6d',simOMPL.StateSpaceType.pose3d,drone_handle,{-2.5,-2,0},{2.5,2,1.5},1)}
    simOMPL.setStateSpace(t,ss)
    simOMPL.setAlgorithm(t,simOMPL.Algorithm.RRTConnect)
    simOMPL.setCollisionPairs(t,{sim.getObjectHandle('eDrone_outer'),collection_handles})

    --------------------Add your publisher and subscriber nodes here ---------------------

    path_pub=simROS.advertise('/vrep/waypoints', 'geometry_msgs/PoseArray')    -- Publish the computed path under the topic /vrep/waypoints
    whycon_sub = simROS.subscribe('/computepath', 'std_msgs/Bool', 'computecallback')




    -- Hint : You will require to subscribe to a topic published by path_planning.py indicating the need of new path once all waypoints are covered. THIS IS IMPORTANT
    ---------------------------------------------------------------------------------------


    scale_factor = {-7.5580137672097, -7.515183578371, -17.702372189925} -- Add the scale_factor you computed learned from the tutorial of whycon transformation
    no_of_path_points_required = 55

end


-- This function can be used to visualize the path you compute. This function takes path points as the argument...
-- GO through the code segment for better understanding
function visualizePath( path )
    if not _lineContainer then
        _lineContainer=sim.addDrawingObject(sim.drawing_lines,1,0,-1,99999,{0.2,0.2,1})
    end
    sim.addDrawingObjectItem(_lineContainer,nil)
    if path then
        local pc=#path/7
        for i=1,pc-1,1 do
            lineDat={path[(i-1)*7+1],path[(i-1)*7+2],path[(i-1)*7+3],path[i*7+1],path[i*7+2],path[i*7+3]}
            sim.addDrawingObjectItem(_lineContainer,lineDat)
        end
    end
end


-- This function is used to send the Path computed in the real_world to whycon_world after transformation
-- Message is to be sent in geometry_msgs/PoseArray as WhyCon node subscribes to that message
function packdata(path)

    local sender = {header = {}, poses = {}}

    sender['header']={seq=123, stamp=simROS.getTime(), frame_id="drone"}
    sender['poses'] = {}

    for i=1,((no_of_path_points_required-1)*7)+1,7 do
        a = {x = 0, y = 0, w = 0, z = 0}
        b = {x = 0, y = 0, z = 0}
        pose = {position = b, orientation = a, }

        -------------------Add x, y and z value after converting real_world to whycon_world using the computed scale_factor--------------------------------

        pose.position.x = (path[i]) * scale_factor[1] - 0.02
        pose.position.y = (path[i+1])* scale_factor[2] - 0.02
        pose.position.z = (path[i+2])* scale_factor[3] + 55.60
        sender.poses[math.floor(i/7) + 1] = pose


        --------------------------------------------------------------------------------------------------------------------
    end
    -- Debug if the path computed are correct. Display the computed path and see if the path points moves from drone to the target point
    return sender
end

function getpose(handle,ref_handle)
    position = sim.getObjectPosition(handle,ref_handle)
    orientation = sim.getObjectQuaternion(handle,ref_handle)
    pose = {position[1],position[2],position[3],orientation[1],orientation[2],orientation[3],orientation[4]}
    return pose
end
--- This function is used to compute and publish the path to path_planninglpy
function compute_and_send_path(task)
    local r
    local path

    r,path=simOMPL.compute(t,10,-1,no_of_path_points_required)-- Provide the correct arguments here.. Make sure you check the no of path points it actually computes

    if(r == true) then
        visualizePath(path)
        message = packdata(path)  
        simROS.publish(path_pub,message)

        -- Provide slots for debug to cross check the message or path points you recieve after computing
    end
    return r
end
--funtion for callback

function computecallback(msg)
    if(msg.data==true) then
       if #goals_handles >= 2 then
            compute_path_flag = true
        end     
    end
end
function sysCall_actuation()

    ---- Add your code to set start and goal state after getting present poses of the drone and the new goal when path_planning.py request you a path
    ---------------------------------------------------------------------------------------------------------------
    if compute_path_flag == true then
        -- Getting startpose
        start_pose = getpose(goals_handles[1],-1)
        -- Getting the goalpose
        goal_pose = getpose(goals_handles[2],-1)
        -- Setting start state
        simOMPL.setStartState(t,start_pose)
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        -- Computing path and publishing path points
        status = compute_and_send_path(t)
        if(status == true) then -- path computed
            compute_path_flag = false
            table.remove(goals_handles,1)
        end
    end 




    ----------------------------------------------------------------------------------------------------------------
    --Hint : Set orientation of the goal as same as that of the start pose as you don't want the drone to rotate on its yaw

end


function sysCall_sensing()

end



function sysCall_cleanup()

end
