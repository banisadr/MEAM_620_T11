function minheap()
% test minheap implementation

    a = rand(1,200);

    heap = zeros(size(a));
    n_heap = 0;

    for(i=1:numel(a))
        [heap, n_heap] = minheap_insert(a(i), heap, n_heap);
    end

    heap
    n_heap

    % remove elements
    for(i=1:numel(a))
        [el(i), heap, n_heap] = minheap_extract(heap, n_heap);
    end

    assert(all(el == sort(a)));

end

function [heap, n_heap] = minheap_insert(el, heap, n_heap)
    heap(n_heap+1) = el;
    n_heap = n_heap+1;
    heap = heapify_up(n_heap, heap);
end

function heap = heapify_up(pos, heap)
    parent = floor(pos/2);
    if parent && heap(pos) < heap(parent)
        valpos = heap(pos);
        heap(pos) = heap(parent);
        heap(parent) = valpos;
        heap = heapify_up(parent, heap);
    end
end

function [el, heap, n_heap] = minheap_extract(heap, n_heap)
    el = heap(1);
    heap(1) = heap(n_heap);
    n_heap = n_heap-1;
    heap = heapify_down(1, heap, n_heap);
end

function heap = heapify_down(pos, heap, n_heap)
    childpos = [2*pos, 2*pos+1];
    if n_heap < 2*pos
        return;
    elseif n_heap == 2*pos;
        heap(2*pos+1) = inf;
    end

    if heap(pos) > heap(childpos(1)) || ...
            heap(pos) > heap(childpos(2))
        [minchildval, minchildid] = min([heap(childpos(1)), heap(childpos(2))]);
        heap(childpos(minchildid)) = heap(pos);
        heap(pos) = minchildval;
        heap = heapify_down(childpos(minchildid), heap, n_heap);
    end
end
