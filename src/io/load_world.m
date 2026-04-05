function world = load_world(filename)
    data = load(filename);
    assert(size(data, 2) >= 14, 'world.dat must have at least 14 columns');

    world = struct();
    world.id = data(:, 1);
    world.position = data(:, 2:4);
    world.appearance = data(:, 5:end);
end
