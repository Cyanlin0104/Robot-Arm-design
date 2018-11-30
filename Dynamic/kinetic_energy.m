function Ti = kinetic_energy(w, vc, m, Ic)
Ti = (m*(vc.')*vc)/2 + ((w.')*Ic*w)/2;
end