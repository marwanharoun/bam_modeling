function a = wrapangle(angle_rad) 
    % θ = wrapangle(angle_rad)
    % Return the same angle in radians but in the interval [-π,π].
    a = angle_rad - 2*pi*floor( (angle_rad+pi)./(2*pi) );
end