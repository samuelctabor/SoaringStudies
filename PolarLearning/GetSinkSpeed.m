function Vz=GetSinkSpeed(V, Polar)
    Cl = Polar.k./V.^2;
    Cd =Polar.Cd0 + Polar.B.*Cl.^2;
    Vz = V.*Cd./Cl;
end