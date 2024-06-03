-- Local Modular

-- Synchronize local G
Gloc1 = sync(G1,G3)
Gloc2 = sync(G1,G3)
Gloc3 = sync(G1,G2)
Gloc4 = sync(G1,G2)

-- Synchronize local K
Kloc1 = sync(Gloc1,E1)
Kloc2 = sync(Gloc2,E2)
Kloc3 = sync(Gloc3,E3)
Kloc4 = sync(Gloc4,E4)

-- Create local supervisors
Sloc1 = supc(Gloc1, Kloc1)
Sloc2 = supc(Gloc2, Kloc2)
Sloc3 = supc(Gloc3, Kloc3)
Sloc4 = supc(Gloc4, Kloc4)

print("----------")
print(infom(Kloc1, Kloc2, Kloc3, Kloc4))
print(infom(Sloc1, Sloc2, Sloc3, Sloc4))

Kloc1 = minimize(Kloc1)
Kloc2 = minimize(Kloc2)
Kloc3 = minimize(Kloc3)
Kloc4 = minimize(Kloc4)

Sloc1 = minimize(Sloc1)
Sloc2 = minimize(Sloc2)
Sloc3 = minimize(Sloc3)
Sloc4 = minimize(Sloc4)

print("----------")
print(infom(Kloc1, Kloc2, Kloc3, Kloc4))
print(infom(Sloc1, Sloc2, Sloc3, Sloc4))

-- Add to Nadzoru
export( Sloc1 )
export( Sloc2 )
export( Sloc3 )
export( Sloc4 )
