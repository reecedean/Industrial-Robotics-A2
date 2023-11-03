function [intersectionPoint, check] = LinePlaneIntersection(planeNormal, pointOnPlane,point1OnLine, point2OnLine)

    intersectionPoint = [0, 0, 0];
    u = point2OnLine - point1OnLine;
    w = point1OnLine - pointOnPlane;
    D = dot(planeNormal, u);
    N = -dot(planeNormal, w);
    check = 0;
    if abs(D) < 10^-7
        if N == 0
            check = 2;
            return
        else
            check = 0;
            return
        end
    end

sI = N / D;
intersectionPoint = point1OnLine + sI.*u;

    if (sI < 0 || sI > 1)
        check = 3;
    else 
        check = 1;
    end
end