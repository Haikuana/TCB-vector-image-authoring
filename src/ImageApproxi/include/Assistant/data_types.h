#ifndef _DATA_TYPES_
#define _DATA_TYPES_

#define _USE_MATH_DEFINES

#include <Windows.h>
#include <gl/GLU.h>

#include <CGAL/Cartesian.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_vertex_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/Constrained_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Voronoi_diagram_2.h> 
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/lloyd_optimize_mesh_2.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>
#include <CGAL/Constrained_triangulation_plus_2.h>

#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Partition_is_valid_traits_2.h>
#include <CGAL/polygon_function_objects.h>
#include <CGAL/partition_2.h>

#include <CGAL/convex_hull_2.h>
#include <CGAL/Bbox_3.h>

#include <vector>
#include <list>
#include <set>
#include <string>
#include <assert.h>
#include <algorithm>

#include "omp.h"

#include "Assistant/enriched_polyhedron.h"
#include "Assistant/gl2ps.h"

#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/boost/graph/properties_Polyhedron_3.h>
#define CGAL_EIGEN3_ENABLED
#include <CGAL/Surface_mesh_deformation.h>

#include <QString>
#include <QObject>
#include <ANN.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>  
#include <opencv2/highgui.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/OrderingMethods>

#define PRECISE

#define	PI_ 3.141592653

#define IMAGEWIDTHSIZE 0.5
#define IMAGEGRAYMIN 0.0
#define IMAGEGRAYMAX 0.5
#define NUMFITTINGELEMENT 5

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2                                          Point_2;
typedef K::Point_3                                          Point_3;
typedef K::Vector_2                                         Vector_2;
typedef K::Vector_3                                         Vector_3;
typedef K::Segment_2 Segment_2;
typedef K::Line_2 Line_2;
typedef K::Iso_rectangle_2 Iso_rectangle_2;
typedef K::Ray_2 Ray_2;
typedef K::Triangle_2 Triangle_2;

typedef CGAL::Cartesian<double>  Kernel3;
typedef Kernel3::Iso_cuboid_3 Iso_cuboid;
typedef CGAL::Simple_cartesian<double> Simple_cartesian;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Bbox_2 Bbox_2;
typedef CGAL::Bbox_3 Bbox_3;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;

typedef CGAL::Exact_predicates_exact_constructions_kernel	K_exact;
typedef K_exact::Point_2									Point_e2;
typedef K_exact::Point_3									Point_e3;
typedef CGAL::Polygon_2<K_exact>							Polygon_e2;
typedef CGAL::Polygon_with_holes_2<K_exact>					Polygon_with_holes_e2;

typedef Eigen::VectorXd VectorXd;
typedef Eigen::MatrixXd MatrixXd;
typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::Triplet<double> Tri;
typedef Eigen::SimplicialLDLT<SpMat> SimplicialLDLT;
typedef Eigen::AMDOrdering<int> AMDOrdering;
typedef Eigen::SparseQR<SpMat, AMDOrdering> QR;
typedef Eigen::SparseLU<SpMat, AMDOrdering> LU;

typedef CGAL::Cartesian<double>  Kernel3;
typedef Enriched_polyhedron<Kernel3,Enriched_items> Mesh;
typedef Mesh::Vertex_iterator Vertex_iterator;
typedef Mesh::Vertex_const_iterator Vertex_const_iterator;
typedef Mesh::Facet_iterator Facet_iterator;
typedef Mesh::Face_handle Face_handle;
typedef Mesh::Halfedge_around_facet_circulator Halfedge_around_facet_circulator;
typedef Mesh::Iso_cuboid Iso_cuboid;
typedef Mesh::Point Point;
typedef Mesh::Vector Normal;

using std::vector;
using std::map;
using std::list;
using std::pair;
using std::set;

#define BOUNDARYSIZE 3

#define CONSIDER_G1_CURVE 0

struct color
{
	double r;
	double g;
	double b;
};

struct VertexPair
{
	Point_2 pt2;
	Point_3 pt3;
};

struct PointTriple
{
	int index;//arranged index
	int index_x;//pixel index
	int index_y;//pixel index
	double curvature;//discrete curvature
	Point_2 imagep;//-0.5-0.5		original position
	Point_2 image_newp;//-0.5-0.5	smoothing position
	Point_2 imagedomain;//0-1		smoothing on domain
	Point_2 paradomain;//projected 0-1	para position
	bool	is_fixed;//for some situations
	bool	is_headortail;//for some situations
	Mesh::Vertex_iterator vit;//link mesh points
	double constant_size;//=200.0/inputimage width
};

struct SurfaceTangentForG1
{
	int index_pos;
	Point_2 domain_pos;
	Point_2 dir1;
	Point_2 dir2;
	map<int, double> mergebasis_derivate_value;
};

struct RuptureKnotConfig
{
	vector<unsigned int> inte;
	pair<int,int> constrain;//for corner basis: second is corner
	int split_spare_index=-1;//for corner basis split by broken segment
	bool bInverse_order = false;
};

struct PolygonPairs
{
	vector<VertexPair> vertexPairs;
};

struct FaceInfo
{
	int facetindex;
	bool bUpdate;
	double cost;
	double weight;
	vector<unsigned> indices;
	vector<Vertex_iterator> incInfos;
	vector<Point_2> triangle;
};

struct CellVertexPair
{
	double density;
	Point_2 pt2;
};

struct CellPairs
{
	vector<CellVertexPair> vertexPairs;
};

class CellInfo
{
public:
	void clear_data()
	{
		incInfos.clear();
		faceInfos.clear();
		polygons.clear();
	}
	vector<Vertex_iterator> incInfos;
	vector<Face_handle> faceInfos;
	vector<CellPairs> polygons;
};

struct PlaneCoefficients
{
	double a;
	double b;
	double c;
};

struct NeighbourFaceInfo
{
	bool bExist;
	FaceInfo info;
};

struct BoundaryDist
{
	double x0Dist;
	double y0Dist;
	double xnDist;
	double ynDist;
};

struct Point2withIndex
{
	Point_2 point2;
	int id;//for itself
	int index;//for others
	int spare_index;
	bool is_updated;
};

struct KnotFlag{
	union 
	{
		unsigned whole[2];
		struct
		{
			unsigned bNew: 1;
			unsigned bCorner: 1;
			unsigned bFeature: 1;
			unsigned bMulti: 1;
			unsigned nAdd: 6;
			unsigned xIndex: 12;
			unsigned yIndex: 12;
			unsigned oriIndex: 14;
			unsigned bBoundary: 16;
		};
	};
};

typedef struct _CvtKnot
{
	double x;
	double y;
	double intx;
	double inty;
	double intd;
	vector<unsigned> neighbourIds;
	vector<Point_2> cell;
	KnotFlag flag; //1-8th bit is on boundaries; 9th bit: is new knot? 10th: is feature?
	bool bcellUpdate;
}CvtKnot;

class ImageIndex
{
public:
	int xIndex;
	int yIndex;
	int index;
	ImageIndex()
	{
		xIndex = 0;
		yIndex = 0;
		index = 0;
	}
	bool operator<(const ImageIndex& id)
	{
		if (yIndex < id.yIndex)
		{
			return true;
		}
		else if(yIndex == id.yIndex)
		{
			return xIndex < id.xIndex;
		}
		else
		{
			return false;
		}
	}
	bool operator==(const ImageIndex& id)
	{
		return index == id.index;
	}
};

typedef struct _DualCell
{
	unsigned index; 
	vector<Point_2> vertices;
}DualCell;

struct PolygonsInCell
{
	Point_2 center;
	double area;
	double density;
};

typedef struct _BoundaryPolygon
{
	vector<Point_2> boundaries;
	Point_2 center;
	Vector_2 size;
}BoundaryPolygon;

typedef struct Link_
{
	list<unsigned> sortEdges;
	list<pair<unsigned, unsigned>> orgEdges;
	list<pair<unsigned, unsigned>> edges;
	vector<unsigned> interior;
}Link;

typedef struct KLink_
{
	list<Link> links;
	int order;
}KLink;

enum Triangulation_Type
{
	DELAUNAY,
	DDT
};

enum TriangulationOptimizationAlg
{
	LOP,
	LOOKAHEAD,
	SIMULATEDANNEALING
};

class EdgeInfo
{
public:
	EdgeInfo(unsigned id1_, unsigned id2_)
	{
		id1 = id1_;
		id2 = id2_;
	}
	unsigned id1;
	unsigned id2;
	bool operator < (const EdgeInfo &edge) const
	{
		if(id1<edge.id1)
			return true;
		else
		{
			if(id1>edge.id1)
				return false;
			else
				return id2<edge.id2;
		}
	} 
};

struct KnotData
{
	Point_2 pt;
	unsigned index;
	KnotFlag flag;
};

class SupportInfo
{
public:
	unsigned index;
	Point_2 pt;
	SupportInfo()
	{
		index = 0;
		pt = Point_2(0, 0);
	}
	SupportInfo(unsigned id, Point_2 p)
	{
		index = id;
		pt = p;
	}
	bool operator < (const SupportInfo &support) const
	{
		if(index<support.index)
			return true;
		else
			return false;
	} 
	bool operator==(const SupportInfo &support) const
	{
		if(index==support.index)
			return true;
		else
			return false;
	} 
};

struct RGBPoint
{
	double x;
	double y;
	double r = 0.78;//0-1
	double g=0.89;//0-1
	double b=0.93;//0-1
	bool is_on_feature;
	bool is_on_bound;
	int basis_index;//in basisConfigs rather than basisMerges
};

struct DomainValue
{
	double value;
	double dx;
	double dy;
	double dxx;
	double dxy;
	double dyy;
	RGBPoint surfacep;
	Point_2 pt;

	//extra data
	Point_2 dir1;
	Point_2 dir2;
	double value_dir1;
	double value_dir2;
};

struct G1Corner
{
	vector<unsigned> inte_side1;
	vector<unsigned> inte_cen;
	vector<unsigned> inte_side2;
	int index_side1;
	int index_side2;
	int index_cen_pair1;
	int index_cen_pair2;
};

struct PixelAttribute
{
	pair<int, int> coord;
	double	error_;
	double	err_spotsum;
	bool	do_select;
	int		vindex;
	int		insert_status;
};

typedef vector<unsigned> TConfig1;

struct TConfig2//one simplex
{
	TConfig1					tconfig;
	int							allconfigs_index;
	double						tri_area;
};

struct TConfig3//simplex with domain data
{
	bool						bUsed;
	vector<unsigned>			tconfig;
	map<unsigned, DomainValue>	supports;
	unsigned					index;
};

struct SplineBasisConfigs
{
	vector<unsigned>			config;//interior 
	vector<TConfig2>			tconfigs;
	map<unsigned, DomainValue>	supports;
	vector<double>				initial_points;
	RGBPoint					controlPt;
	bool						bCCWorientation;//used for control points pair; true for first p; false for second;
};


class ErrorTriangleInfo
{
public:
	ErrorTriangleInfo()
	{
		triangle[0] = Point_2(-1, -1);
		triangle[1] = Point_2(-1, -1);
		triangle[2] = Point_2(-1, -1);
		maxError = 0;
		totalError = 0;
		center = Point_2(-1, -1);
	}
	Point_2 &operator[](int index)
	{
		index = (index+3)%3;
		return triangle[index];
	}
	bool operator>(const ErrorTriangleInfo &info) const
	{
		return maxError > info.maxError;
	}
	double &get_max_error()
	{
		return maxError;
	}
	double &get_total_error()
	{
		return totalError;
	}
	Point_2 &get_center()
	{
		return center;
	}
private:
	Point_2 triangle[3];
	double maxError;
	double totalError;
	Point_2 center;
};

struct OneStepInfo
{
	double mean_error;
	double mean_err_nofeature;
	double SNR;
	double MSE;
	double RMSE;
	double MSE_nofea;
	double psnr;
	double mssim;
	double compression_ratio;

	int num_knots;
	int num_knots_face;
	int num_controlp;
	double size_representation;
	
	double t_feature_process;
	double t_para;
	double t_knots_generation;
	double t_LTP;
	double t_basis_config;
	double t_basis_value;
	double t_verify;
	double t_sample_data;
	double t_solve_equation;
	double t_fairing;
	double t_timeall;
};

struct StatisticsInfos
{
	vector<OneStepInfo> infos;
	double allTime;
};

struct BasisMergeInfo
{
	int index;
	int type;//0: equal 1: represent
	double gsx;//Greville site;
	double gsy;//Greville site;
	vector<unsigned> basisMerge;
};

struct CoplanarInfo
{
	unsigned cornerId;
	unsigned cornerBasisId;
	vector<unsigned> c0Ids;
	Normal normal;
	Normal rnormal;
	Normal gnormal;
	Normal bnormal;
};

struct BSplineBasis
{
	int deg;
	vector<SplineBasisConfigs> basisConfigs;
	vector<BasisMergeInfo> basisMergeInfos;
	vector<CoplanarInfo> cpInfos;
};

struct EqualInfo
{
	std::vector<unsigned> innerIds;
	std::vector<unsigned> index;
};

class CMEdgeInfo
{
private:
	unsigned m_first;
	unsigned m_second;
public:
	CMEdgeInfo()
	{
		m_first = 0;
		m_second = 0;
	}
	CMEdgeInfo(unsigned first, unsigned second)
	{
		m_first = first;
		m_second = second;
	}
	const unsigned &first() const
	{
		return m_first;
	}
	const unsigned &second() const
	{
		return m_second;
	}
	void set_first(unsigned fst)
	{
		m_first = fst;
	}
	void set_second(unsigned snd)
	{
		m_second = snd;
	}
	bool operator ==(const CMEdgeInfo &m)const 
	{
		return (m_first == m.m_first && m_second == m.m_second) &&
			(m_first == m.m_second && m_second == m.m_first);
	}
	bool operator <(const CMEdgeInfo &m)const 
	{
		unsigned min_elem = std::min(m_first, m_second);
		unsigned mmin_elem = std::min(m.first(), m.second());
		if(min_elem<mmin_elem)
			return true;
		else if (min_elem==mmin_elem)
		{
			unsigned max_elem = std::max(m_first, m_second);
			unsigned mmax_elem = std::max(m.first(), m.second());
			if (max_elem<mmax_elem)
				return true;
			else
				return false;
		}
		else//>
			return false;
	}
};

struct CMVertexInfo
{
	vector<unsigned> interior;
	vector<unsigned> triangle_config;
	RGBPoint vertex;
	pair<RGBPoint, RGBPoint> control_point_pair;
	pair<RGBPoint, RGBPoint>  cpp_little_trans;// little translation for control points pair
	Point_2 center;
	int merge_basis_id;
	pair<int, int> feature_mergebasis_id;
	bool is_on_feature;
	int flag; //-1 not processed 0: dummy, 1: real
	int feacp_type;//type for control points on feature, value correspond to the index of feature sequences
	bool do_display;
};

struct CMFaceInfo
{
	vector<unsigned> face;
	int flag; //1: configs_1, 2, configs_2
};

struct CMInfo
{
	vector<CMVertexInfo> vertices;
	set<CMEdgeInfo> edges;
	vector<CMFaceInfo> faces;
};

typedef std::pair<int,int> Vpair;

////////////////////////////////////////////////////////
//for edit

struct EditPoint
{
	Vector_2 origin;
	Vector_2 deviation;
	bool	bselected;
};

struct EditVertex
{
	RGBPoint					cp;
	pair<RGBPoint, RGBPoint>	cp_pair;//for feature case; only has one side(first: real position and rgb; second: moved position)
	bool						is_on_feature;
	bool						is_bound;
	bool						is_updated;
	bool						bdisplay = true;
	map<unsigned, double>		supports;
	Point_2						domain_midEdge;
	bool						bCCWorientation;//used for control points pair; true for first p; false for second;
	RGBPoint					deviation;
	RGBPoint					previous_cp;
	pair<RGBPoint, RGBPoint>	previous_pair;
};

struct EditCM
{
	vector<EditVertex> vertices;
	set<CMEdgeInfo> edges;
};

struct EditFM
{
	vector<RGBPoint> vertices;
	vector<CMFaceInfo> faces;
};

struct AuthoringP2
{
	bool is_cycle = false;
	Point_2 point2;
	int id;//for itself

	RGBPoint v_positive;
	RGBPoint v_negative;
};

struct AuthoringV
{
	RGBPoint					cp;
	RGBPoint					pair;//for feature case; only has one side(first: real position and rgb; second: moved position)
	bool						is_on_feature = false;
	bool						is_bound = false;

	RGBPoint					deviation;
	RGBPoint					previous_cp;
	RGBPoint					previous_pair;

	Vpair						cpseq_id;
};

struct AuthoringCM
{
	vector<AuthoringV> vertices;
	std::set<Vpair> edges;//on feature
	vector<CMFaceInfo> faces;
};

typedef vector<vector<Point2withIndex>> CPs;
typedef vector<vector<AuthoringP2>> ACPs;

/////////////////////////////////////////////////////////

struct SubSet
{
	int loc;
	vector<unsigned> sameIds;
};

struct NearInfo
{
	unsigned index;
	double dist;
};

struct CentroidInfo
{
	Point_2 centroid;
	double cost;
};

enum SplineType
{
	TENSOR,
	SIMPLEX,
	DMS
};

struct PolylineData
{
	Point_3 vertex;
	Point_2 domain;
};

class ColumnSubset
{
public:
	unsigned start;
	unsigned end;
	unsigned newCol;
	double deviation;
	vector<double> curvatures;
	vector<double> arclength;
	bool operator >(const ColumnSubset &m)const 
	{
		if (abs(deviation-m.deviation)>std::numeric_limits<double>::epsilon())
			return deviation>m.deviation;
		else
			return (end-start)>(m.end-m.start);
	}
};

struct UnivariateSpline
{
	vector<unsigned> supports;
	vector<double> values;
	vector<double> knots;
};

struct GridInfo
{
	double xmin;
	double xmax;
	double ymin;
	double ymax;
	double density;
	CellInfo cellInfo;
};

template < class Gt, class Vb = CGAL::Triangulation_vertex_base_2<Gt> >
class Fst_vertex_base : public Vb
{
	typedef Vb Base;
public:
	typedef typename Vb::Vertex_handle Vertex_handle;
	typedef typename Vb::Face_handle Face_handle;
	typedef typename Vb::Point Point;
	template < typename TDS2 >
	struct Rebind_TDS
	{
		typedef typename Vb::template Rebind_TDS<TDS2>::Other Vb2;
		typedef Fst_vertex_base<Gt, Vb2> Other;
	};
private:
	unsigned index = -1;
	Vpair cpseqs_id;
	RGBPoint cp;
	bool bNew;
public:
	Fst_vertex_base() : Base() {}
	Fst_vertex_base(const Point& p) : Base(p) {}
	Fst_vertex_base(const Point& p, Face_handle f) : Base(f, p) {}
	Fst_vertex_base(Face_handle f) : Base(f) {}
	void set_associated_index(unsigned ind) { index = ind; }
	unsigned get_associated_index() { return index; }
	void set_cseq_index(Vpair ind) { cpseqs_id = ind; }
	Vpair get_cseq_index() { return cpseqs_id; }
	void set_cp(RGBPoint p) { cp = p; }
	RGBPoint get_cp() { return cp; }
	void set_new_status(bool bN) { bNew = bN; }
	bool get_new_status() { return bNew; }
};


typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo, K> Fbb;//true:flipped face  false:old face
typedef CGAL::Constrained_triangulation_face_base_2<K, Fbb> Fb;
typedef Fst_vertex_base<K>               Vb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb>   Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds>     CDT;
typedef CGAL::Constrained_triangulation_2<K, Tds>     CT;
typedef CGAL::Delaunay_triangulation_2<K, Tds> DT;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT>                 AT;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT> AP;
typedef CGAL::Voronoi_diagram_2<DT, AT, AP>                                    VD;

typedef CGAL::Delaunay_mesh_face_base_2<K> D_Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, D_Fb> T_ds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, T_ds> CDT_Refine;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT_Refine> Criteria;

typedef CGAL::Exact_predicates_exact_constructions_kernel KE;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo, KE> FbbE;//true:flipped face  false:old face
typedef CGAL::Constrained_triangulation_face_base_2<KE, FbbE> FbE;
typedef Fst_vertex_base<KE>               VbE;
typedef CGAL::Triangulation_data_structure_2<VbE, FbE>   TdsE;
typedef CGAL::Exact_intersections_tag                     Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<KE, TdsE, Itag> CDTE;
typedef CGAL::Constrained_triangulation_plus_2<CDTE>      CDT_Intersect;
typedef CDT_Intersect::Point                              Point_Intersect;

struct  BaryPoint
{
	double lamda0;
	double lamda1;
	double lamda2;
	Point_2 p;
	CDT_Intersect::Vertex_handle vhandle;
	int index;
};

typedef pair<CT::Vertex_handle, CT::Vertex_handle> Edge;

struct FaceSort
{
	vector<unsigned> face;
	CT::Face_iterator fit;
};

#endif