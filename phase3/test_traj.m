function tests = test_traj
    tests = functiontests(localfunctions);
end

function check_start_goal(testcase, map, start, stop)
    path = {dijkstra(map, start, stop, true)};
    init_script;
    trajectory = test_trajectory({start}, {stop}, map, path, false);
    close all;

   verifyEqual(testcase, trajectory{1}(1,1:3), start, 'AbsTol', 1e-2);
   verifyEqual(testcase, trajectory{1}(end,1:3), stop, 'AbsTol', 1e-2);
end

function test_mapclean(testcase)
    map = load_map('phase2/sample_maps/map_clean.txt', 0.1, 2.0, 0.3);

    % start in centroid, end in centroid
    check_start_goal(testcase, map, [0.55  0.55 3.], [0.65  0.55 3.]);
    % start in centroid, end not in centroid
    check_start_goal(testcase, map, [0.55  0.55 3.], [0.7  0.7 3.]);
    % not start in centroid, end in centroid
    check_start_goal(testcase, map, [0.7  0.7 3.], [0.55  0.55 3.]);
    % not start nor end in centroid
    check_start_goal(testcase, map, [0.7  0.7 3.], [0.9  0.9 3.]);

    % start = stop
    check_start_goal(testcase, map, [0.7  0.7 3.], [0.7  0.7 3.]);
    check_start_goal(testcase, map, [0.75  0.75 3.], [0.75  0.75 3.]);
    check_start_goal(testcase, map, [0.75  0.75 3.], [0.7  0.7 3.]);
    check_start_goal(testcase, map, [0.7  0.7 3.], [0.75  0.75 3.]);
    % start, stop on same cell
    check_start_goal(testcase, map, [0.7  0.7 3.], [0.73  0.73 3.5]);
end

function test_nopath(testcase)
    map = load_map('phase2/sample_maps/map_clean.txt', 0.1, 2.0, 0.3);
    path = {[0.55  0.55 3.]};
    init_script;
    trajectory = test_trajectory({[0.55  0.55 3.]}, {[0.65  0.55 3.]}, map, path, false);
    % TODO: what we want to test here? if dijkstra return empty traj we don't even know
    %       the start and finishing points.
end
