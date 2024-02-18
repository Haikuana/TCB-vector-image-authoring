///////////////////////////////////////////////////////////////////////////
//                                                                       //
//  Class: Enriched_polyhedron                                           //
//                                                                       //
///////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef _POLYGON_MESH_
#define _POLYGON_MESH_

// CGAL stuff
#include <list>
#include <algorithm>
#include <QString>
#include <QTextStream>

#include <CGAL/Cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#define GL_POLYGON                        0x0009

struct StatusFlag
{
	union 
	{
		unsigned whole;
		struct
		{
			unsigned bSelected: 1;
			unsigned bStatus: 2; 
			unsigned preserve: 29;
			bool	bshow;
		};
	};
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2                                          Point_2;
typedef K::Point_3                                          Point_3;

// a refined facet with a normal and a tag
template <class Refs, class T, class P, class Norm>
class Enriched_facet : public CGAL::HalfedgeDS_face_base<Refs, T>
{
  //attributions
  int m_tag; 
  Norm m_normal;
  double d_area;
  double m_area;
  double density;
  double error;
  int index;
  Point_2 error_doaminp;
  StatusFlag flag; // 0 unprocessed, 1, processed, 2,not selected
public:
  Enriched_facet()
  {
  }

  // tag
  const int&  tag() { return m_tag; }
  void tag(const int& t)  { m_tag = t; }

  // normal
  typedef Norm Normal_3;
  Normal_3& normal() { return m_normal; }
  const Normal_3& normal() const { return m_normal; }
  double& area(){return m_area;}
  const double& area() const {return m_area;}

  int &facet_index()
  {
	  return index;
  }

  StatusFlag &facet_flag()
  {
	  return flag;
  }

  double &facet_density()
  {
	  return density;
  }

  double &domain_area()
  {
	  return d_area;
  }

  Point_2 &error_knot()
  {
	  return error_doaminp;
  }

  double &facet_error()
  {
	  return error;
  }

};

// a refined halfedge with a general tag and 
// a binary tag to indicate whether it belongs 
// to the control mesh or not
template <class Refs, class Tprev, class Tvertex, class Tface, class Norm>
class Enriched_halfedge : public CGAL::HalfedgeDS_halfedge_base<Refs,Tprev,Tvertex,Tface>
{
private:
  // tag
  int m_tag; 

  // option for edge superimposing
  bool m_control_edge; 

  int bnormal;
  bool status;
public:
	void set_status(bool s)
	{
		status = s;
	}
	bool &get_status()
	{
		return status;
	}

  // life cycle
  Enriched_halfedge()
  {
    m_control_edge = true;
  }

  //
  int &is_normal()
  {
	  return bnormal;
  }

  // tag
  const int& tag() const { return m_tag;  }
  int& tag() { return m_tag;  }
  void tag(const int& t)  { m_tag = t; }

  // control edge 
  bool& control_edge()  { return m_control_edge; }
  const bool& control_edge()  const { return m_control_edge; }
  void control_edge(const bool& flag) { m_control_edge  = flag; }
};



// a refined vertex with a normal and a tag
template <class Refs, class T, class P, class Norm>
class Enriched_vertex : public CGAL::HalfedgeDS_vertex_base<Refs, T, P>
{
private:
  // predicate
  int					m_tag; //feature class 0:non 1:yes 2:adjacent
  bool					border;
  bool					bcorner;
  bool					bfeature;
  int					featuretype;
  bool					bused;
  bool					bcomputecurvature;
  bool					bMaxError;

  //attribute
  double				g_curvature;	//gaussian curvature
  double				m_curvature;	//mean curvature
  double				areas;
  Point_3				color;
  Point_3				pixel_color;
  Point_3				input_color;
  double				error;
  double				density;
  Norm					m_normal;		// normal
  StatusFlag			flag;			//0: in Polygon, 1: intersect the Polygon, 2: other cases

  //position
  int					index;
  int					index_standby;
  std::pair<int, int>	coordinate;		//coordinate
  Point_2				domain;     
  Point_2				old_domain;
  Point_2				smooth_point_;

public:
  // life cycle
  Enriched_vertex()  
  {
	  color = Point_3(0.75, 0.75, 0.75);
  }
  // repeat mandatory constructors
  Enriched_vertex(const P& pt)
    : CGAL::HalfedgeDS_vertex_base<Refs, T, P>(pt)
  {
	  color = Point_3(0.75,0.75,0.75);
  }

  bool &is_border()
  {
	  return border;
  }
  bool &is_corner()
  {
	  return bcorner;
  }

  StatusFlag &vertex_flag()
  {
	  return flag;
  }

  bool &is_feature()
  {
	  return bfeature;
  }

  int &feature_type()
  {
	  return featuretype;
  }

  bool &is_used()
  {
	  return bused;
  }

  bool &is_max_error()
  {
	  return bMaxError;
  }

  double &gaussian_curvature()
  {
	  return g_curvature;
  }

  double &mean_curvature()
  {
	  return m_curvature;
  }

  int &vertex_index()
  {
	  return index;
  }
 
  int &vertex_standby_index()
  {
	  return index_standby;
  }

  std::pair<int,int> &vertex_coordinate()
  {
	  return coordinate;
  }

  Point_2 &smooth_point()
  {
	  return smooth_point_;
  }

  Point_2 &get_domain()
  {
	  return domain;
  }

  Point_2 &get_old_domain()
  {
	  return old_domain;
  }

  Point_3 &vertex_color()
  {
	  return color;
  }

  Point_3 &vertex_pixel_color()
  {
	  return pixel_color;
  }

  Point_3 &vertex_input_color()
  {
	  return input_color;
  }

  double &vertex_density()
  {
	  return density;
  }

  double &vertex_error()
  {
	  return error;
  }

  // normal
  typedef Norm Normal_3;
  Normal_3& normal() { return m_normal; }
  const Normal_3& normal() const { return m_normal; }

  // tag
  int& tag() {  return m_tag; }
  const int& tag() const {  return m_tag; }
  void tag(const int& t)  { m_tag = t; }
  
};

// A redefined items class for the Polyhedron_3 
// with a refined vertex class that contains a 
// member for the normal vector and a refined
// facet with a normal vector instead of the 
// plane equation (this is an alternative 
// solution instead of using 
// Polyhedron_traits_with_normals_3).

struct Enriched_items : public CGAL::Polyhedron_items_3
{
    // wrap vertex
    template <class Refs, class Traits>
    struct Vertex_wrapper
    {
        typedef typename Traits::Point_3  Point;
        typedef typename Traits::Vector_3 Normal;
        typedef Enriched_vertex<Refs,
                          CGAL::Tag_true,
                          Point,
                          Normal> Vertex;
    };

    // wrap face
    template <class Refs, class Traits>
    struct Face_wrapper
    {
        typedef typename Traits::Point_3  Point;
        typedef typename Traits::Vector_3 Normal;
        typedef Enriched_facet<Refs,
                         CGAL::Tag_true,
                         Point,
                         Normal> Face;
    };

    // wrap halfedge
    template <class Refs, class Traits>
    struct Halfedge_wrapper
    {
        typedef typename Traits::Vector_3 Normal;
        typedef Enriched_halfedge<Refs,
                            CGAL::Tag_true,
                            CGAL::Tag_true,
                            CGAL::Tag_true,
                            Normal> Halfedge;
    };
};

//*********************************************************
template <class kernel, class items>
class Enriched_polyhedron : public CGAL::Polyhedron_3<kernel,items>
{
public :
  typedef typename kernel::FT FT;
  typedef typename kernel::Point_3 Point;
  typedef typename kernel::Vector_3 Vector;
  typedef typename kernel::Iso_cuboid_3 Iso_cuboid;
  //typedef typename items::Vertex_wrapper::Normal Normal;

private :

  // bounding box
  Iso_cuboid	m_bbox;

  // type
  bool			m_pure_quad;
  bool			m_pure_triangle;
  double		m_maxDens;
  double		m_minDens;
  double		m_max_meancurvature;
  double		m_min_meancurvature;
  double		maxError;
  double		minError;
  double		rmse;
  int			nVBorder;
  int			nVCorner;
  double		boxLength;
  std::vector<Vertex_iterator> vertex_ite;
  std::vector<Vertex_iterator> vertex_ite_standby;
  std::vector<Halfedge_iterator> halfedge_ite;
  std::vector<Facet_iterator> facet_ite;

public :
	
	void compute_all_face_domain_area()
	{
		Facet_iterator fit = facets_begin();
		for (; fit!=facets_end(); fit++)
		{
			Halfedge_around_facet_circulator pHalfedge = fit->facet_begin(), start = pHalfedge;
			int j=0;
			vector<Point_2> pts;
			do 
			{
				pts.push_back(pHalfedge->vertex()->get_domain());
			} while (++pHalfedge!=start);
			fit->domain_area() = std::abs(CGAL::area(pts[0], pts[1], pts[2]));
		}
	}

	Vertex_iterator  get_vertex_iterator(int s)
	{
		return vertex_ite[s];
	}

	Vertex_iterator  get_vertex_standby_iterator(int s)
	{
		return vertex_ite_standby[s];
	}

	Halfedge_iterator get_halfedge_iterator(int s)
	{
		return halfedge_ite[s];
	}

	Facet_iterator get_facet_iterator(int s)
	{
		return facet_ite[s];
	}

	void set_all_edges_status(bool flag)
	{
		Edge_iterator eit = edges_begin();
		for (; eit!=edges_end(); eit++)
		{
			eit->set_status(flag);
		}
	}

	void set_all_halfedges_status(bool flag)
	{
		Halfedge_iterator hit = halfedges_begin();
		for (; hit != halfedges_end(); hit++)
		{
			hit->set_status(flag);
		}
	}

	void set_all_vertices_selected(int s)
	{
		Vertex_iterator vit = vertices_begin();
		for (; vit!=vertices_end(); vit++)
		{
			vit->vertex_flag().bSelected = s;
		}
	}

	void set_all_vertices_status(int s)
	{
		Vertex_iterator vit = vertices_begin();
		for (; vit!=vertices_end(); vit++)
		{
			vit->vertex_flag().bStatus = s;
		}
	}

	void set_all_vertices_density(double *rhos)
	{
		Vertex_iterator vit = vertices_begin();
		int i=0;
		for (; vit!=vertices_end(); vit++, i++)
		{
			vit->vertex_density() = rhos[i];
		}
	}
	void set_all_facets_selected(int s)
	{
		Facet_iterator fit = facets_begin();
		for (; fit!=facets_end(); fit++)
		{
			fit->facet_flag().bSelected = s;
		}	
	}

	void set_all_facets_status(int s)
	{
		Facet_iterator fit = facets_begin();
		for (; fit!=facets_end(); fit++)
		{
			fit->facet_flag().bStatus = s;
		}	
	}

	void initial_mesh_status()
	{
		Vertex_iterator vit = vertices_begin();
		int i=0;
		for (; vit!=vertices_end(); vit++, i++)
		{
			vit->vertex_flag().whole = 0;
		}
		Facet_iterator fit = facets_begin();
		for (; fit!=facets_end(); fit++)
		{
			fit->facet_flag().whole = 0;
		}	
	}
	
	int getNumVerticesBorder()
	{
		return nVBorder;
	}

	int getNumVerticesCorner()
	{
		return nVCorner;
	}

	void computeBorderVerticesType()
	{
		nVBorder = 0;
		Vertex_iterator vit = vertices_begin();
		for(; vit!=vertices_end(); vit++)
		{
			//border
			if(is_border(vit))
			{
				vit->is_border() = true;
				nVBorder++;
			}
			else
			{
				vit->is_border() = false;
			}	
		}
	}

	void computeCornerVerticesType()
	{
		nVCorner = 0;
		//corner
		Vertex_iterator vit = vertices_begin();
		for (; vit != vertices_end(); vit++)
		{
			if (abs(vit->get_domain().x() - 0) < 1e-8 &&abs(vit->get_domain().y() - 0) < 1e-8 ||
				abs(vit->get_domain().x() - 0) < 1e-8 &&abs(vit->get_domain().y() - 1) < 1e-8 ||
				abs(vit->get_domain().x() - 1) < 1e-8 &&abs(vit->get_domain().y() - 0) < 1e-8 ||
				abs(vit->get_domain().x() - 1) < 1e-8 &&abs(vit->get_domain().y() - 1) < 1e-8)
			{
				vit->is_corner() = true;
				nVCorner++;
			}
			else
			{
				vit->is_corner() = false;
			}
		}
	}

  void set_max_min_density(double &maxD, double &minD)
  {
	  m_maxDens = maxD;
	  m_minDens = minD;
  }

  double &get_max_density()
  {
	  return m_maxDens;
  }
  double &get_min_density()
  {
	  return m_minDens;
  }

  void set_max_min_curvature(double &maxc, double &minc)
  {
	  m_max_meancurvature = maxc;
	  m_min_meancurvature = minc;
  }

  double &get_max_curvature()
  {
	  return m_max_meancurvature;
  }
  double &get_min_curvature()
  {
	  return m_min_meancurvature;
  }

  void set_max_error(double e)
  {
	  maxError = e;
  }

  double get_max_error()
  {
	  return maxError;
  }

  void set_min_error(double e)
  {
	  minError = e;
  }

  double get_min_error()
  {
	  return minError;
  }

  void set_rmse(double e)
  {
	  rmse = e;
  }

  double &get_box_length()
  {
	  return boxLength;
  }

  double get_rmse()
  {
	  return rmse;
  }
  // life cycle
  Enriched_polyhedron() 
  {
	  m_pure_quad = false;
	  m_pure_triangle = false;
	  m_maxDens = 0.0;
	  m_minDens = 0.0;
	  m_max_meancurvature = 0.0;
	  m_min_meancurvature = 0.0;
	  maxError = 0.0;
	  minError = 0.0;
	  rmse = 0.0;
	  nVBorder = 0;
	  boxLength = 0.0;
	  vertex_ite.clear();
	  halfedge_ite.clear();
	  facet_ite.clear();
  }

  //Enriched_polyhedron(Enriched_polyhedron *pmesh) :Polyhedron_3<kernel, items>(pmesh)
  //{
	 // //vertex attribute
	 // Vertex_iterator vit = vertices_begin(), vt = pmesh->vertices_begin();
	 // for (; vit != vertices_end(); vit++, vt++)
	 // {
		//  vit->vertex_flag() = vt->vertex_flag();
		//  vit->is_border() = vt->is_border();
		//  vit->is_corner() = vt->is_corner();
		//  vit->is_feature() = vt->is_feature();
		//  vit->is_used() = vt->is_used();
		//  vit->is_max_error() = vt->is_max_error();
		//  vit->gaussian_curvature() = vt->gaussian_curvature();
		//  vit->mean_curvature() = vt->mean_curvature();
		//  vit->vertex_index() = vt->vertex_index();
		//  vit->vertex_standby_index() = vt->vertex_standby_index();
		//  vit->vertex_coordinate() = vt->vertex_coordinate();
		//  vit->get_domain() = vt->get_domain();
		//  vit->get_old_domain() = vt->get_old_domain();
		//  vit->vertex_color() = vt->vertex_color();
		//  vit->vertex_pixel_color() = vt->vertex_pixel_color();
		//  vit->vertex_density() = vt->vertex_density();
		//  vit->vertex_error() = vt->vertex_error();
		//  vit->tag() = vt->tag();
		//  vit->normal() = vt->normal();
	 // }
	 // //halfedge attribute
	 // Halfedge_iterator hit = halfedges_begin(), ht = pmesh->halfedges_begin();
	 // for (; hit != halfedges_end(); hit++, ht++)
	 // {
		//  hit->set_status(ht->get_status());
		//  hit->tag() = ht->tag();
	 // }

	 // //face attribute
	 // Facet_iterator fit = facets_begin(), ft = pmesh->facets_begin();
	 // for (; fit != facets_end(); fit++, ft++)
	 // {
		//  fit->normal() = ft->normal();
		//  fit->tag(ft->tag());
		//  fit->area() = ft->area();
		//  fit->facet_index() = ft->facet_index();
		//  fit->facet_flag() = ft->facet_flag();
		//  fit->facet_density() = ft->facet_density();
		//  fit->domain_area() = ft->domain_area();
		//  fit->facet_error() = ft->facet_error();
	 // }

	 // m_pure_quad = false;
	 // m_pure_triangle = false;
	 // m_maxDens = 0.0;
	 // m_minDens = 0.0;
	 // m_max_meancurvature = 0.0;
	 // m_min_meancurvature = 0.0;
	 // maxError = 0.0;
	 // minError = 0.0;
	 // rmse = 0.0;
	 // nVBorder = 0;
	 // boxLength = 0.0;
	 // vertex_ite.clear();
	 // halfedge_ite.clear();
	 // facet_ite.clear();
	 // m_bbox = pmesh->bbox();
  //}


  void copy_attribute(Enriched_polyhedron *pmesh)
  {
	  //vertex attribute
	  Vertex_iterator vit = vertices_begin(),vt = pmesh->vertices_begin();
	  for (; vit != vertices_end(); vit++,vt++)
	  {
		  vit->vertex_flag() = vt->vertex_flag();
		  vit->is_border() = vt->is_border();
		  vit->is_corner() = vt->is_corner();
		  vit->is_feature() = vt->is_feature();
		  vit->is_used() = vt->is_used();
		  vit->is_max_error() = vt->is_max_error();
		  vit->gaussian_curvature() = vt->gaussian_curvature();
		  vit->mean_curvature() = vt->mean_curvature();
		  vit->vertex_index() = vt->vertex_index();
		  vit->vertex_standby_index() = vt->vertex_standby_index();
		  vit->vertex_coordinate() = vt->vertex_coordinate();
		  vit->get_domain() = vt->get_domain();
		  vit->get_old_domain() = vt->get_old_domain();
		  vit->vertex_color() = vt->vertex_color();
		  vit->vertex_pixel_color() = vt->vertex_pixel_color();
		  vit->vertex_density() = vt->vertex_density();
		  vit->vertex_error() = vt->vertex_error();
		  vit->tag() = vt->tag();
		  vit->normal() = vt->normal();
	  }
	  //halfedge attribute
	  Halfedge_iterator hit = halfedges_begin(), ht = pmesh->halfedges_begin();
	  for (; hit != halfedges_end(); hit++, ht++)
	  {
		  hit->set_status(ht->get_status());
		  hit->tag() = ht->tag();
	  }

	  //face attribute
	  Facet_iterator fit = facets_begin(), ft = pmesh->facets_begin();
	  for (; fit != facets_end(); fit++, ft++)
	  {
		  fit->normal() = ft->normal();
		  fit->tag(ft->tag());
		  fit->area() = ft->area();
		  fit->facet_index() = ft->facet_index();
		  fit->facet_flag() = ft->facet_flag();
		  fit->facet_density() = ft->facet_density();
		  fit->domain_area() = ft->domain_area();
		  fit->facet_error() = ft->facet_error();
	  }
	  //mesh attribute
	  bbox() = pmesh->bbox();
  }

  virtual ~Enriched_polyhedron() 
  {

  }

  // type
  bool is_pure_triangle() { return m_pure_triangle; }
  bool is_pure_quad() { return m_pure_quad; }

  void compute_index()
  {
	  Vertex_iterator start = vertices_begin();
	  int i=0;
	   for(Vertex_iterator pVertex = start; pVertex != vertices_end();  pVertex++)
	   {
		   pVertex->vertex_index() = i;//std::distance(start, pVertex);
		   i++;
	   }
	   Facet_iterator fit = facets_begin();
	   i = 0;
	   while (fit!=facets_end())
	   {
		   fit->facet_index() = i;
		   fit++;
		   i++;
	   }
  }
  //caution: need recreate when copy
  void map_vertex_index_to_iterator()
  {
	  vertex_ite.resize(size_of_vertices());
	  for (Vertex_iterator pVertex = vertices_begin(); pVertex != vertices_end(); pVertex++)
	  {
		  vertex_ite[pVertex->vertex_index()] = pVertex;
	  }
  }
  //caution: need recreate when copy
  void map_vertex_standby_index_to_iterator()
  {
	  vertex_ite_standby.resize(size_of_vertices());
	  for (Vertex_iterator pVertex = vertices_begin(); pVertex != vertices_end(); pVertex++)
	  {
		  vertex_ite_standby[pVertex->vertex_standby_index()] = pVertex;
	  }
  }
  //caution: need recreate when copy
  void map_halfedge_index_to_iterator()
  {
	  halfedge_ite.resize(size_of_halfedges());
	  for (Halfedge_iterator h_it = halfedges_begin(); h_it != halfedges_end(); h_it++)
	  {
		  halfedge_ite[h_it->halfedge_index()] = h_it;
	  }
  }
  //caution: need recreate when copy
  void map_facet_index_to_iterator()
  {
	  facet_ite.resize(size_of_facets());
	  for (Facet_iterator f_it = facets_begin(); f_it != facets_end(); f_it++)
	  {
		  facet_ite[f_it->facet_index()] = f_it;
	  }
  }

  // normals (per facet, then per vertex)
  void compute_normals_per_facet()
  {
    std::for_each(facets_begin(),facets_end(),Facet_normal());
  }
  void compute_normals_per_vertex()
  {
    std::for_each(vertices_begin(),vertices_end(),Vertex_normal());
  }
  void compute_normals()
  {
    compute_normals_per_facet();
    compute_normals_per_vertex();
  }

  // bounding box
  Iso_cuboid& bbox() { return m_bbox; }
  const Iso_cuboid bbox() const { return m_bbox; }

  // compute bounding box
  void compute_bounding_box()
  {
    if(size_of_vertices() == 0)
    {
      //Q_ASSERT(false);
      return;
    }

    FT xmin,xmax,ymin,ymax,zmin,zmax;
    Vertex_iterator pVertex = vertices_begin();
    xmin = xmax = pVertex->point().x();
    ymin = ymax = pVertex->point().y();
    zmin = zmax = pVertex->point().z();
    for(;
        pVertex !=  vertices_end();
        pVertex++)
    {
      const Point& p = pVertex->point();

      xmin =  std::min<FT>(xmin,p.x());
      ymin =  std::min<FT>(ymin,p.y());
      zmin =  std::min<FT>(zmin,p.z());

      xmax =  std::max<FT>(xmax,p.x());
      ymax =  std::max<FT>(ymax,p.y());
      zmax =  std::max<FT>(zmax,p.z());
    }
    m_bbox = Iso_cuboid(xmin,ymin,zmin,
                        xmax,ymax,zmax);
	boxLength = std::max<FT>(xmax-xmin, ymax-ymin);
	boxLength = std::max<FT>(boxLength, zmax-zmin);
  }

  // bounding box
  FT xmin() { return m_bbox.xmin(); }
  FT xmax() { return m_bbox.xmax(); }
  FT ymin() { return m_bbox.ymin(); }
  FT ymax() { return m_bbox.ymax(); }
  FT zmin() { return m_bbox.zmin(); }
  FT zmax() { return m_bbox.zmax(); }

  // copy bounding box
  void copy_bounding_box(Enriched_polyhedron<kernel,items> *pMesh)
  {
    m_bbox = pMesh->bbox();
  }

  // degree of a face
  static unsigned int degree(Facet_handle pFace)
  {
    return CGAL::circulator_size(pFace->facet_begin());    
  }

  // valence of a vertex
  static unsigned int valence(Vertex_handle pVertex)
  {
    return CGAL::circulator_size(pVertex->vertex_begin());
  }

  // check whether a vertex is on a boundary or not
  static bool is_border(Vertex_handle pVertex)
  {
    Halfedge_around_vertex_circulator pHalfEdge = pVertex->vertex_begin();
    if(pHalfEdge == NULL) // isolated vertex
      return true;
    Halfedge_around_vertex_circulator d = pHalfEdge;
    CGAL_For_all(pHalfEdge,d)
      if(pHalfEdge->is_border())
        return true;
    return false;
  }

  // get any border halfedge attached to a vertex
  Halfedge_handle get_border_halfedge(Vertex_handle pVertex)
  {
    Halfedge_around_vertex_circulator pHalfEdge = pVertex->vertex_begin();
    Halfedge_around_vertex_circulator d = pHalfEdge;
    CGAL_For_all(pHalfEdge,d)
      if(pHalfEdge->is_border())
        return pHalfEdge;
    return NULL;
  }

  // tag all halfedges
  void tag_halfedges(const int tag)
  {
    for(Halfedge_iterator pHalfedge = halfedges_begin();
        pHalfedge != halfedges_end();
        pHalfedge++)
      pHalfedge->tag(tag);
  }

  // tag all facets
  void tag_facets(const int tag)
  {
    for(Facet_iterator pFacet = facets_begin();
        pFacet  != facets_end();
        pFacet++)
      pFacet->tag(tag);
  }

  // set index for all vertices
  void set_index_vertices()
  {
    int index = 0;
    for(Vertex_iterator pVertex = vertices_begin();
        pVertex != vertices_end();
        pVertex++)
      pVertex->tag(index++);
  }

  // is pure degree ?
  bool is_pure_degree(unsigned int d)
  {
    for(Facet_iterator pFace  = facets_begin();
        pFace != facets_end();
        pFace++)
      if(degree(pFace) != d)
        return false;
    return true;
  }

  // compute type
  void compute_type()
  {
    m_pure_quad = is_pure_degree(4);
    m_pure_triangle = is_pure_degree(3);
  }

  // compute facet center
  void compute_facet_center(Facet_handle pFace,
                            Point& center)
  {
    Halfedge_around_facet_circulator pHalfEdge = pFace->facet_begin();
    Halfedge_around_facet_circulator end = pHalfEdge;
    Vector vec(0.0,0.0,0.0);
    int degree = 0;
    CGAL_For_all(pHalfEdge,end)
    {
      vec = vec + (pHalfEdge->vertex()->point()-CGAL::ORIGIN);
      degree++;
    }
    center = CGAL::ORIGIN + (vec/(kernel::FT)degree);
  }

  void write_obj_with_curvature_domain_color(QString fileName, bool bmean_curvature, bool bgauss_curvature, bool bdomain, bool bcolor)
  {
	  QFile file(fileName);
	  if (file.open(QIODevice::WriteOnly))
	  {
		  QTextStream out(&file);
		  out << "####" << endl;
		  out << "#" << endl;
		  out << "# OBJ File Generated by Simplex Spline" << endl;
		  out << "#" << endl;
		  QStringList list = fileName.split("/");
		  if (list.size() == 1)
		  {
			  list = fileName.split("\\");
		  }
		  out << "# Object " << list.at(list.size() - 1) << endl;
		  out << "#" << endl;
		  out << "# Vertices: " << size_of_vertices() << endl;
		  out << "# Faces: " << size_of_facets() << endl;
		  out << "#" << endl;
		  out << "####" << endl;

		  Vertex_iterator v_it = vertices_begin();
		  for (; v_it != vertices_end(); v_it++)
		  {
			  out << "v " << v_it->point().x() << " " << v_it->point().y() << " " << v_it->point().z() << endl;
			  if (bmean_curvature)
			  {
				  out << "m " << v_it->mean_curvature() << endl;
			  }
			  if (bgauss_curvature)
			  {
				  out << "g " << v_it->gaussian_curvature() << endl;
			  }
			  if (bdomain)
			  {
				  out << "d " << v_it->get_domain().x() << " " << v_it->get_domain().y() << endl;
			  }
			  if (bcolor)
			  {
				  out << "c " << v_it->vertex_color().x() << " " << v_it->vertex_color().y() << " " << v_it->vertex_color().z() << endl;
			  }
			  else
				  out << endl;
		  }

		  Facet_iterator f_it = facets_begin();
		  for (; f_it != facets_end(); f_it++)
		  {
			  out << "f ";
			  Halfedge_around_facet_circulator pHalfedge = f_it->facet_begin();
			  int i = 0;
			  do
			  {
				  if (is_pure_triangle())
				  {
					  if (i == 2)
						  out << pHalfedge->vertex()->vertex_index() + 1 << endl;
					  else
						  out << pHalfedge->vertex()->vertex_index() + 1 << " ";
				  }
				  else
				  {
					  if (i == 3)
						  out << pHalfedge->vertex()->vertex_index() + 1 << endl;
					  else
						  out << pHalfedge->vertex()->vertex_index() + 1 << " ";
				  }
				  i++;
			  } while (++pHalfedge != f_it->facet_begin());
		  }
	  }
  }

  void write_obj_with_vtype(QString fileName)
  {
	  QFile file(fileName);
	  if (file.open(QIODevice::WriteOnly))
	  {
		  QTextStream out(&file);
		  out << "####" << endl;
		  out << "#" << endl;
		  out << "# OBJ File Generated by Simplex Spline" << endl;
		  out << "#" << endl;
		  QStringList list = fileName.split("/");
		  if (list.size() == 1)
		  {
			  list = fileName.split("\\");
		  }
		  out << "# Object " << list.at(list.size() - 1) << endl;
		  out << "#" << endl;
		  out << "# Vertices: " << size_of_vertices() << endl;
		  out << "# Faces: " << size_of_facets() << endl;
		  out << "#" << endl;
		  out << "####" << endl;

		  Vertex_iterator v_it = vertices_begin();
		  for (; v_it != vertices_end(); v_it++)
		  {
			  out << "v " << v_it->point().x() << " " << v_it->point().y() << " " << v_it->point().z();
			  out << " vc " << v_it->vertex_input_color().x() << " " 
				  << v_it->vertex_input_color().y() << " " 
				  << v_it->vertex_input_color(). z();
			  {
				  int bound_type = -1;
				  if (v_it->is_border())
				  {
					  if (abs(v_it->get_domain().x() - 0) < 1e-8)
					  {
						  bound_type = 0;
					  }
					  else  if (abs(v_it->get_domain().x() - 1.0) < 1e-8)
					  {
						  bound_type = 1;
					  }
					  else  if (abs(v_it->get_domain().y() - 0) < 1e-8)
					  {
						  bound_type = 2;
					  }
					  else  if (abs(v_it->get_domain().y() - 1.0) < 1e-8)
					  {
						  bound_type = 3;
					  }
				  }
				  
				  out << " vt " <<v_it->is_corner()<<" "<<
					  bound_type <<" "<< v_it->is_feature() << endl;
			  }
		  }

		  Facet_iterator f_it = facets_begin();
		  for (; f_it != facets_end(); f_it++)
		  {
			  out << "f ";
			  Halfedge_around_facet_circulator pHalfedge = f_it->facet_begin();
			  int i = 0;
			  do
			  {
				  if (is_pure_triangle())
				  {
					  if (i == 2)
						  out << pHalfedge->vertex()->vertex_index() + 1 << endl;
					  else
						  out << pHalfedge->vertex()->vertex_index() + 1 << " ";
				  }
				  else
				  {
					  if (i == 3)
						  out << pHalfedge->vertex()->vertex_index() + 1 << endl;
					  else
						  out << pHalfedge->vertex()->vertex_index() + 1 << " ";
				  }
				  i++;
			  } while (++pHalfedge != f_it->facet_begin());
		  }
	  }
  }

  void write_obj(QString fileName, bool color)
  {
	QFile file( fileName);
	if ( file.open(QIODevice::WriteOnly)) 
	{
		QTextStream out( &file );
		out << "####" << endl;
		out << "#" << endl;
		out <<"# OBJ File Generated by Simplex Spine" <<endl;
		out << "#" << endl;
		QStringList list = fileName.split("/");
		if(list.size()==1)
		{
			list = fileName.split("\\");
		}
		out << "# Object " << list.at(list.size()-1) << endl;
		out << "#" << endl;
		out << "# Vertices: " << size_of_vertices() << endl;
		out << "# Faces: " << size_of_facets() << endl;
		out << "#" <<endl;
		out << "####" <<endl;
		
		Vertex_iterator v_it = vertices_begin();
		for(; v_it!=vertices_end(); v_it++)
		{
			out << "v " << v_it->point().x() << " " << v_it->point().y() << " " << v_it->point().z() << " ";
			if(color)
			{
				out << v_it->vertex_color().x() << " " << v_it->vertex_color().y() << " " << v_it->vertex_color().z() << endl;
			}
			else
				out << endl;
		}

		Facet_iterator f_it = facets_begin(); 
		for(; f_it!=facets_end(); f_it++)
		{
			out << "f ";
			Halfedge_around_facet_circulator pHalfedge = f_it->facet_begin();
			int i=0;
			do
			{
				if(is_pure_triangle())
				{
					if(i==2)
						out << pHalfedge->vertex()->vertex_index()+1 << endl;
					else
						out << pHalfedge->vertex()->vertex_index()+1 <<  " ";
				}
				else
				{
					if(i==3)
						out << pHalfedge->vertex()->vertex_index()+1 << endl;
					else
						out << pHalfedge->vertex()->vertex_index()+1 << " ";
				}
				i++;
			}while(++pHalfedge!=f_it->facet_begin());
		}
	}
  }
};

// compute facet normal 
struct Facet_normal // (functor)
{
  template <class Facet>
  void operator()(Facet& f)
  {
    typename Facet::Normal_3 sum = CGAL::NULL_VECTOR;
    typename Facet::Halfedge_around_facet_circulator h = f.facet_begin();
    typename Facet::Normal_3 normal = CGAL::cross_product(
        h->next()->vertex()->point() - h->vertex()->point(),
        h->next()->next()->vertex()->point() - h->next()->vertex()->point());
    double sqnorm = std::sqrt(normal * normal);
	f.area() = sqnorm/2;
    if(sqnorm >std::numeric_limits<double>::epsilon())
      f.normal() = normal / sqnorm;
    else
    {
      f.normal() = CGAL::NULL_VECTOR;
      //TRACE("degenerate face\n");
    }
  }
};


// compute vertex normal 
struct Vertex_normal // (functor)
{
    template <class Vertex>
    void operator()(Vertex& v)
    {
        typename Vertex::Normal_3 normal = CGAL::NULL_VECTOR;
		double area_sum = 0;
        Vertex::Halfedge_around_vertex_const_circulator pHalfedge = v.vertex_begin();
        Vertex::Halfedge_around_vertex_const_circulator begin = pHalfedge;
        do
		{
			if (pHalfedge->facet()!=NULL)
			{
				const double m_area = pHalfedge->facet()->area();
				area_sum += m_area; 
				normal = normal + pHalfedge->facet()->normal()*m_area;
			}
	    }while(++pHalfedge!=begin);
		normal = normal/area_sum;
        double sqnorm = normal * normal;
        if(sqnorm > std::numeric_limits<double>::epsilon())
          v.normal() = normal / std::sqrt(sqnorm);
        else
          v.normal() = CGAL::NULL_VECTOR;
    }
};

#endif // _POLYGON_MESH_
