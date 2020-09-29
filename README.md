# RV-seminarski
Seminarski iz RV

Tema:Vizuelna odometrija na osnovu podataka prikupljenih RGB-D senzorom   
Kao ground-truth je korišten Carla simulator  
Princip rada:  
U svakom frejmu se na osnovu podatka o dubini formira point cloud sa koordinatama izraženim u koordinatnom sistemu kamere. Na RGB slici se vrši detekcija značajki, i uparivanje sa pripadnim značajkama iz prethodnog frame-a. Za to se koristi Lucas-Kanade tracking algoritam koji za opisivanje značajki koristi Shi-Thomasi deskriptor za detekciju cornera.  

Ideja je da se primjenni 3D-3D algoritam za estimaciju pozicije, tj. određuje optimalno R i t koje minimizira normu
