function tests = collide_test
tests = functiontests(localfunctions);
end

function assertNoCollision(testcase, map, points)
c = collide(map, points);
verifyEqual(testcase, size(points, 1), length(c(:)));
verifyFalse(testcase, any(c));
end

function assertCollision(testcase, map, points)
c = collide(map, points);
verifyTrue(testcase, isvector(c));
verifyEqual(testcase, size(points, 1), length(c(:)));
verifyTrue(testcase, all(c));
end

function testMap0_1(testcase)
map = load_map('sample_maps/map0.txt', 0.2, 0.5, 0.2);

valid = [0.0  -1.0 2.0; 
         3.0  17.0 4.0; 
         0.0  -5.0 0.5];
collision = [0.0 2.0 1.0; 
             3.0 18.5 4.5];
assertNoCollision(testcase, map, valid);
assertCollision(testcase, map, collision);
end

function testMap0_2(testcase)
xy_res = 0.2;
z_res = 0.5;
margin = 0.2;
map = load_map('sample_maps/map0.txt', xy_res, z_res, margin);

collision = [map.block(:,1:3);
             map.block(:,4:6)];
assertCollision(testcase, map, collision);
end

%% TODO write more tests!
% point at center of each box
% points at corners of each box
% points right in/out of margin