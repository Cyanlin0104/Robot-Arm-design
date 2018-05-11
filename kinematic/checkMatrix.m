function bool = checkMatrix(m1, m2)
    bool = sum(sum(m1)) == sum(sum(m2));
end
