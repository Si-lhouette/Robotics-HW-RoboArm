function Q = getQ(n_seg, n_order, ts)
    Q = [];
    for k = 1:n_seg
        Q_k = [];
        %#####################################################
        % STEP 1.1: calculate Q_k of the k-th segment 
        np = n_order + 1;
        Q_k = zeros(np,np);
        for i = 3:n_order
            for l = 3:n_order
                Q_k(i+1,l+1) = (i)*(i-1)*(i-2)*l*(l-1)*(l-2)/(i+l-5) * ts(k)^(i+l-5);
            end
        end
        Q = blkdiag(Q, Q_k);
    end
end