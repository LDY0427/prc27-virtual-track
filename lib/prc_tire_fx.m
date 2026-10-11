function Fx = prc_tire_fx(P, kappa, Fz)
% PRC_TIRE_FX  종방향 타이어 힘 (단순 매직포뮬러). Fx = Fz·μ·sin(C·atan(Bκ − E(Bκ − atan Bκ)))
T = P.tire;
Bk = T.B * kappa;
Fx = Fz .* T.mu .* sin(T.C * atan(Bk - T.E * (Bk - atan(Bk))));
end
