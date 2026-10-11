function lt = prc_lambda_target(P, tps_pct)
% PRC_LAMBDA_TARGET  TPS(%)에 따른 목표 람다 (P.map.lambda_target_tps/val 선형 보간).
lt = interp1(P.map.lambda_target_tps, P.map.lambda_target_val, ...
             min(max(tps_pct, P.map.lambda_target_tps(1)), P.map.lambda_target_tps(end)));
end
