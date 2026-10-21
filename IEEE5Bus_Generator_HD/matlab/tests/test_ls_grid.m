function [pass, msg] = test_ls_grid()
%Farkli H,D degerleri ve her makine icin kendi Pm basamagi ile LS dogrulugu.
sets = {[2;5.5;9], [0.5;1.5;5]; [9;2;5.5], [5;0.5;1.5]};
worstH = 0; worstD = 0;
for s = 1:size(sets,1)
    for k = 1:3
        mult = ones(1,3); 
        o = case5SwingSim(struct('H', sets{s,1}, 'D', sets{s,2}, 'Tend', 10, ...
            'pmFun', @(t) 1 + 0.2*(t >= 1)*double((1:3) == k)));
        r = estimateHD_LS(o.t, o.Tm(:,k), o.Te(:,k), o.w(:,k), 1);
        worstH = max(worstH, abs(r.H - o.H(k))/o.H(k));
        worstD = max(worstD, abs(r.D - o.D(k))/o.D(k));
    end
end
pass = worstH < 0.03 && worstD < 0.05;
msg = sprintf('6 sim x makine: en kotu bagil hata H=%.2f%%, D=%.2f%%', 100*worstH, 100*worstD);
end
