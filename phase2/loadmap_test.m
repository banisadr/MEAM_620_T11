function tests = loadmap_test
    tests = functiontests(localfunctions);
end

function check_edge_len(testcase, map)
    sizes = [map.cost_dx, map.cost_dy, map.cost_dz, ...
             norm([map.cost_dx, map.cost_dy]), ...
             norm([map.cost_dx, map.cost_dz]), ...
             norm([map.cost_dz, map.cost_dy])];
    for(i=1:prod(map.dims))
        [px, py, pz] = ind2sub(map.dims, i);
        pos_i = [map.x(px), map.y(py), map.z(pz)];
        neighbors = find(map.graph(:,i));
        [px, py, pz] = ind2sub(map.dims, neighbors);
        pos_n = [map.x(px)', map.y(py)', map.z(pz)'];
        dists = pdist2(pos_i, pos_n);
        err = pdist2(dists',sizes');
        verifyTrue(testcase, all(any(err < 1e-6,2)));
    end
end


function testMap0(testcase)
    map = load_map('sample_maps/map0.txt', 1, 1, .3);
    check_edge_len(testcase, map);
    map = load_map('sample_maps/map0.txt', .2, 1, .3);
    check_edge_len(testcase, map);
    map = load_map('sample_maps/map0.txt', .2, 4, .3);
    check_edge_len(testcase, map);
end


function testMap1(testcase)
    map = load_map('sample_maps/map1.txt', 1, 1, .3);
    check_edge_len(testcase, map);
    map = load_map('sample_maps/map1.txt', .2, 1, .3);
    check_edge_len(testcase, map);
    map = load_map('sample_maps/map1.txt', .2, 4, .3);
    check_edge_len(testcase, map);
end