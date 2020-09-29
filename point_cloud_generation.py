
import logging
import os
import math
import numpy

from carla.client import make_carla_client
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.image_converter import to_rgb_array, depth_to_array
from carla import sensor
from numpy.matlib import repmat


def depth_to_point_cloud(image):
    
    far = 1000.0  
    normalized_depth = depth_to_array(image)

    # Matrica kamere
    K = numpy.identity(3)
    K[0, 2] = image.width / 2.0  #koordinate ishodista ravni slike se postavljaju u sredinu
    K[1, 2] = image.height / 2.0 #pixel koordinatnog sistema
    K[0, 0] = K[1, 1] = image.width / \
        (2.0 * math.tan(image.fov * math.pi / 360.0)) #relacija iz predavanja 2 koja se lahko izvodi

    # 2d pixel koordinate
    pixel_number = image.width * image.height
    u = repmat(numpy.r_[image.width-1:-1:-1],
                     image.height, 1).reshape(pixel_number)
    v = repmat(numpy.c_[image.height-1:-1:-1],
                     1, image.width).reshape(pixel_number)

    normalized_depth = numpy.reshape(normalized_depth, pixel_number)

    # p2d = [u,v,1]
    p2d = numpy.array([u, v, numpy.ones_like(u)])
    # P = [X,Y,Z]
    p3d = numpy.dot(numpy.linalg.inv(K), p2d)
    p3d *= normalized_depth * far
   
    # [[X1,Y1,Z1],[X2,Y2,Z2], ... [Xn,Yn,Zn]]
    return sensor.PointCloud(image.frame_number, numpy.transpose(p3d))


def run_carla_client(host, port):
    number_of_frames = 80
    output_folder = '_out'
    image_size = [800, 600]
    camera_position = [0.3, 0.0, 1.3] # [X, Y, Z]
    camera_rotation = [0, 0, 0]  # [pitch(Y), yaw(Z), roll(X)]
    fov = 60

    # Connect with the server
    with make_carla_client(host, port) as client:
        print('CarlaClient connected')

        # Here we load the settings.
        settings = CarlaSettings()
        settings.set(
            SynchronousMode=True,
            SendNonPlayerAgentsInfo=False,
            NumberOfVehicles=0,
            NumberOfPedestrians=0,
	    WeatherId=1) 

        camera1 = sensor.Camera('CameraDepth', PostProcessing='Depth', FOV=fov)
        camera1.set_image_size(*image_size)
        camera1.set_position(*camera_position)
        camera1.set_rotation(*camera_rotation)
        settings.add_sensor(camera1)
 
        camera2 = sensor.Camera('CameraRGB', PostProcessing='SceneFinal', FOV=fov)
        camera2.set_image_size(*image_size)
        camera2.set_position(*camera_position)
        camera2.set_rotation(*camera_rotation)
        settings.add_sensor(camera2)

        client.load_settings(settings)

        client.start_episode(1)

        #transformacija KS kamere u KS automobila koja je fiksna jer se ta dva KS zajedno krecu
        #Hcc = camera2.get_unreal_transform() 
	#print(Hcc)  #rotira je po roll za -90 i yaw za 90, prije slanja u transform funkciju
	

        #moze se preskociti prvih 20-ak frameova dok se ono auto ne ustabili nakon respawn-a
        for frame in range(1, number_of_frames):
            # Citas podatke sa servera, pri cemu server slike salje kao BGRA niz bajta
            # Measurement se koristi samo za genersianje sljedecih komandi autopilota
            measurements, sensor_data = client.read_data()
		
	    if frame > 20:

	            # RGB image [[[r,g,b],..[r,g,b]],..[[r,g,b],..[r,g,b]]]
	            image_RGB = to_rgb_array(sensor_data['CameraRGB'])

	            # 2d u camera 3D
		    #inace ga nema potrebe racunati ovdje, nego cemo u vsc, ali ovo je bilo zbog testa
	            
	            point_cloud = depth_to_point_cloud(sensor_data['CameraDepth'])
	            # Odredjivanje point clouda u ovom programu je uvedeno samo zbog testiranja, a odredjivat ce se u glavnom programu gdje se 			    # obradjuju dobijeni frame-ovi
	            # Point cloud iz KS kamere mozemo prebaciti u KS automobila, ali nema potrebe 
      
	            #point_cloud.apply_transform(Hcc)


	            # Spasava .ply file 3D tacaka (mogu se dodati i koristenjem RGB slike) koji se moze vizualizirati npr. u meshlabu
 	 	   
            
	            point_cloud.save_to_disk(os.path.join(output_folder, '{:0>5}.ply'.format(frame)))
            

	            # Spasavanje RGB i depth slika
		    out_filename_format = '_out/{:s}/{:0>5d}'
	    	    for name, measurement in sensor_data.items():
	            	filename = out_filename_format.format(name, frame)
	                measurement.save_to_disk(filename)

	    # Trenutna lokacija vozila, koju mozemo upisati u neki file, kako bi mogli porediti sa estimiranim
	    player_measurements = measurements.player_measurements
	    print('(x,y) = (' + str(player_measurements.transform.location.x) + ',' + str(player_measurements.transform.location.y) + ')')

            client.send_control(measurements.player_measurements.autopilot_control)

#def pose_estimation():
	


def main():

    host = 'localhost'
    port = 2000

    log_level = logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', host, port)
    
    while True:
        try:
            run_carla_client(host=host, port=port)
            print('\nDone!')
            return

        except TCPConnectionError as error:
            logging.error(error)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        print('\nClient stoped by user.')
