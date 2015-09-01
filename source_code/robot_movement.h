#ifndef __ROBOT_MOVEMENT__
#define __ROBOT_MOVEMENT__

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/algorithm.h>
#include <boost/array.hpp>
#include <CGAL/Boolean_set_operations_2.h>


struct FaceInfo2
{
  
  int nesting_level;
  bool visited;
  bool is_constr;
  bool in_domain(){ 
    return nesting_level%2 == 1;
  }
  FaceInfo2(){
  	is_constr=false;

  }
};

typedef CGAL::Exact_predicates_exact_constructions_kernel       K;
typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2,K>    Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K,Fbb>        Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>               TDS;
typedef CGAL::Exact_predicates_tag                                Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>  CDT;
typedef CDT::Point                                                Point;
typedef CGAL::Polygon_2<K>                                        Polygon_2;
typedef K::Point_2 													Point2;

typedef CDT::Face_handle Face_handle;
typedef CDT::Finite_faces_iterator Finite_faces_iterator;
typedef CDT::Point  Point;



void mark_domains(CDT& );
void insert_polygon(CDT& ,const Polygon_2& );
int check_inside(Point2 , Point2 *, Point2 *, K );
bool locate_in_neighbourhood(CDT& ,Face_handle ,Point& ,int );


#endif /*__ROBOT_MOVEMENT__*/
