# RV-seminarski
Seminarski iz RV

Tema:Vizuelna odometrija na osnovu podataka prikupljenih RGB-D senzorom   
Kao ground-truth je korišten Carla simulator  
Princip rada:  
U svakom frejmu se na osnovu podatka o dubini formira point cloud sa koordinatama izraženim u koordinatnom sistemu kamere. Na RGB slici se vrši detekcija značajki, i uparivanje sa pripadnim značajkama iz prethodnog frame-a. Za to se koristi Lucas-Kanade tracking algoritam koji za opisivanje značajki koristi Shi-Thomasi deskriptor za detekciju cornera.  
Ideja je da se primjeni 3D-3D algoritam za estimaciju pozicije, tj. određuje optimalno R i t koje minimizira normu  
Druga ideja, koja bi bila bolja u ovom slučaju je da se koristi neki od algoritama match-ovanja point cloud-a, kao što je ICP.  
Pri realizacij su u velikoj mjeri korištene funkcije i klase koje su dio Carla instalacije, i koje se mogu naci u Carla folderu ovog repozitorija

