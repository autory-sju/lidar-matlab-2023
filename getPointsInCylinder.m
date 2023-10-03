function pointsInCylinder = getPointsInCylinder(center, points)
    indicies = findPointsInCylinder( ...
        points,0.2,Center=center);
    pointsInCylinder = select(points, indicies);
end

