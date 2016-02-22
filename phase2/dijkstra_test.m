function tests = dijkstra_test
tests = functiontests(localfunctions);
end

function assertPathValid(testcase, map, path, start, stop, xlim, ylim, zlim)
valid = xlim(1) <= path(:, 1) & path(:, 1) <= xlim(2);
valid = ylim(1) <= path(:, 2) & path(:, 2) <= ylim(2) & valid;
valid = zlim(1) <= path(:, 3) & path(:, 3) <= zlim(2) & valid;
verifyTrue(testcase, all(valid));

c = collide(map, path);
verifyFalse(testcase, any(c));

verifyEqual(testcase, path(1, :), start);
verifyEqual(testcase, path(end, :), stop);
end

function [path_a, path_d] = compareDijkstraAstar(testcase, map, start, stop, xlim, ylim, zlim)
    [path_a, n_a] = dijkstra(map, start, stop, true);
    [path_d, n_d] = dijkstra(map, start, stop);
    assertPathValid(testcase, map, path_a, start, stop, xlim, ylim, zlim);
    assertPathValid(testcase, map, path_d, start, stop, xlim, ylim, zlim);
    verifyTrue(testcase, n_a <= n_d); % a-star expands less nodes
    pathlen = @(p) sum(sqrt(sum(diff(p).^2,2)));
    verifyEqual(testcase, pathlen(path_a), pathlen(path_d), 'AbsTol', 1e-6); % a-star expands less nodes
    verifyTrue(testcase, n_a <= n_d); % a-star expands less nodes
    verifyTrue(testcase, size(path_a,1) == size(path_d,1)); % both are optimal

end

function testMap0(testcase)
map = load_map('sample_maps/map0.txt', 0.1, 2.0, 0.3);
start = [0.0  -4.9 0.2];
stop  = [8.0  18.0 3.0];
compareDijkstraAstar(testcase, map, start, stop, [0 10], [-5 20], [0 6]);
end

function testMap0_2(testcase)
map = load_map('sample_maps/map0.txt', 0.1, 1.0, 0.3);
start = [5 1.2 4];
stop = [5 14 4];
compareDijkstraAstar(testcase, map, start, stop, [0 10], [-5 20], [0 6]);
end

function testMap0_3(testcase)
% several impossible paths

% start inside block
map = load_map('sample_maps/map0.txt', 0.1, 1.0, 0.3);
start = [5 17 .5];
stop = [5 14 4];
path = dijkstra(map, start, stop, true);
verifyTrue(testcase, isempty(path));

% ends inside block
start = [5 1.2 4];
stop = [5 18.5 5];
path = dijkstra(map, start, stop, true);
verifyTrue(testcase, isempty(path));

% starts outside boundary
start = [-5 1.2 4];
stop = [5 14 4];
path = dijkstra(map, start, stop, true);
verifyTrue(testcase, isempty(path));

% ends outside boundary
start = [5 1.2 4];
stop = [5 25 4];
path = dijkstra(map, start, stop, true);
verifyTrue(testcase, isempty(path));
end


function testMap1_1(testcase)
map = load_map('sample_maps/map1.txt', 0.2, 1.0, 0.3);
start = [0,0,5];
stop = [19,0,5];
compareDijkstraAstar(testcase, map, start, stop, [0 20], [0 5], [0 6]);
end

function testMap1_2(testcase)
map = load_map('sample_maps/map1.txt', 0.2, 1.0, 0.3);

% these are all impossible
startpts = [(map.block(:,1) + map.block(:,4))/2 ...
            (map.block(:,2) + map.block(:,5))/2 ...
            (map.block(:,3) + map.block(:,6))/2];
for i=size(startpts,1)
    start = startpts(i,:);
    stop = [19,0,5];
    path = dijkstra(map, start, stop, true);
    verifyTrue(testcase, isempty(path));
end
end

function testMapClean_1(testcase)
map = load_map('sample_maps/map_clean.txt', 0.2, 1.0, 0.3);
starts = [0,0,5;
          5,2,3;
          2,4,3;
          6,0,0;
          20,5,6];

stops = [1,3,1;
         5,5,6;
         4,5,5;
         20,5,6;
         0,0,0];

for(i=1:size(starts,1))
    start = starts(i,:);
    stop = stops(i,:);
    pathlen = @(p) sum(sqrt(sum(diff(p).^2,2)));
    [path_a, path_d] = compareDijkstraAstar(testcase, map, start, stop, [0 20], [0 5], [0 6]);
    % verifyEqual(testcase, pathlen(path_a), sum(abs(start-stop)), 'AbsTol', 1e-6);
    % verifyEqual(testcase, pathlen(path_d), sum(abs(start-stop)), 'AbsTol', 1e-6);
end

end
