function result = norm_row(input)
    result = sum(input.^2,2).^(0.5);
end