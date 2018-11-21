//
//  coord_map.h
//  MAIN
//
//  Created by Changhyeon Park on 06/09/2018.
//

#ifndef coord_map_h
#define coord_map_h

//// Coordinates Map Data
//typedef struct coordinates_map
//{
//    double latitude, longitude, heading;
//    double x = 0;
//    double y = 0;
//    int WP_address = 0;
//    int WP_next_address = 0;
//    double WP_target_speed = 0;
//    bool WP_stop_line_flag = 0;
//}coordinates_map;

typedef struct coordinates_map
{
    double latitude, longitude, heading;
    double x;
    double y;
    int WP_stop_line_address;
    double WP_stop_line_latitude, WP_stop_line_longitude;
    double WP_stop_line_x;
    double WP_stop_line_y;
    int WP_target_speed;
    
    //    int WP_address = 0;             // WP_stop_line_address 로 수정
    //    int WP_next_address = 0;        // WP_stop_line_x       로 수정
    //    bool WP_stop_line_flag = 0;     // WP_target_speed      로 수정
}coordinates_map;

extern coordinates_map *coord_map;


#endif /* coord_map_h */
