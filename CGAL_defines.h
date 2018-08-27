#ifndef INC_2_3_CGAL_DEFINES_H
#define INC_2_3_CGAL_DEFINES_H

#include <CGAL/Cartesian.h>
#include <CGAL/Gmpq.h>
#include <CGAL/Point_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/basic.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Arr_vertical_decomposition_2.h>
#include <CGAL/Arr_landmarks_point_location.h>
#include <CGAL/graph_traits_Dual_Arrangement_2.h>
#include <CGAL/Arr_face_index_map.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_observer.h>
#include <CGAL/Polygon_with_holes_2.h>

typedef typename CGAL::Gmpq Number_type;
typedef typename CGAL::Cartesian<Number_type> Kernel;
typedef typename Kernel::FT FT;
typedef typename Kernel::Point_2 Point_2;
typedef typename Kernel::Segment_2 Segment_2;
typedef typename Kernel::Line_2 Line_2;
typedef typename CGAL::Polygon_2<Kernel> Polygon_2;
typedef typename CGAL::Polygon_with_holes_2<Kernel> Polygon_with_holes_2;

typedef CGAL::Polygon_set_2<Kernel> Polygon_set_2;
typedef Polygon_set_2::Arrangement_2 Arrangement_2;

typedef typename Polygon_2::Vertex_const_iterator               VrtxCIter;

typedef typename Arrangement_2::Face_handle                     Face_handle;
typedef typename Arrangement_2::Vertex_handle                   Vertex_handle;
typedef typename Arrangement_2::Vertex_const_handle             Vertex_const_handle;
typedef typename Arrangement_2::Face_const_handle               Face_const_handle;
typedef typename Arrangement_2::Face_handle                     Face_handle;
typedef typename Arrangement_2::Halfedge_handle                 Halfedge_handle;
typedef typename Arrangement_2::Halfedge_const_handle           Halfedge_const_handle;
typedef typename Arrangement_2::X_monotone_curve_2              X_monotone_curve_2;
typedef typename Arrangement_2::Vertex_const_iterator           Arr_VrtxCIter;
typedef typename Arrangement_2::Ccb_halfedge_const_circulator   ccb_haledge_circulator;
typedef typename Arrangement_2::Face_iterator                   Face_iterator;

typedef typename CGAL::Arr_landmarks_point_location<Arrangement_2> Landmarks_pl;


#endif //INC_2_3_CGAL_DEFINES_H8
