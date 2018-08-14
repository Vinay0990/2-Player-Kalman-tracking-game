function gain = interGain(ex, ey)
if(ex>ey)
   gain = 0.51;
else 
        gain = 2*abs(ey-ex)/(ex + 0.00001);
end
end