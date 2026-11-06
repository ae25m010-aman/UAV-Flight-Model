function P=FM1_Aux(phase,in)
av=in.av;
P_base=av.P_FC + av.P_GPS + av.P_RX + av.P_telem + av.P_camera + av.P_video;
duty=av.duty.(phase);
I_load=(av.I_maxload - av.I_run) * (av.tau_each * av.load_frac) / av.tau_stall;
I_servo=av.I_idle + duty * (av.I_run + I_load);
P_servo=sum(I_servo) * av.V;
P=(P_base + P_servo) / av.eta_BEC;
end
