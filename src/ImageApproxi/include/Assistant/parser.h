#pragma once
#ifndef PARSER_OBJ_H
#define PARSER_OBJ_H

#include <CGAL/Polyhedron_incremental_builder_3.h>
#include "Assistant/enriched_polyhedron.h"
#include <QString>
#include <QFile>

template <class HDS>
class Builder_obj : public CGAL::Modifier_base<HDS>
{
private:
	typedef typename HDS::Vertex::Point Point;
	typedef typename CGAL::Polyhedron_incremental_builder_3<HDS> Builder;
	Point_3 *vertex;
	int **face;
	int vnum;
	int fnum;
	int fvnum;

public:
	Builder_obj(Point_3 *v, int **f, int vn, int fn, int fv)
	{
		vertex = v;
		face = f;
		vnum = vn;
		fnum = fn;
		fvnum = fv;
	}
	~Builder_obj() {}

	void operator()(HDS& hds)
	{
		Builder builder(hds, true);
		if (fvnum == 4)
			builder.begin_surface(4, 1, 8);
		else
			builder.begin_surface(3, 1, 6);
		read_vertices(builder);
		read_facets(builder);
		builder.end_surface();
	}

private:
	// read vertex coordinates
	void read_vertices(Builder &builder)
	{
		for (int i = 0; i<vnum; i++)
			builder.add_vertex(Point(vertex[i].x(), vertex[i].y(), vertex[i].z()));
	}

	// read facets and uv coordinates per halfedge
	void read_facets(Builder &builder)
	{
		// create facet
		for (int i = 0; i<fnum; i++)
		{
			builder.begin_facet();
			for (int j = 0; j<fvnum; j++)
				builder.add_vertex_to_facet(face[i][j]);
			builder.end_facet();
		}
	}
};

template <class kernel, class items>
class Parser_obj
{
private:
	Point_2 *domain;
public:
	typedef typename Enriched_polyhedron<kernel, items>::HalfedgeDS HalfedgeDS;
	Parser_obj()
	{
		hasParametrization = false;
	}
	~Parser_obj()
	{
		if (!domain)
			delete[]domain;
	}

	Point_2 *get_domain()
	{
		return domain;
	}
	bool hasParametrization;

public:
	bool read(QString &name,
		Enriched_polyhedron<kernel, items> *pMesh)
	{
		CGAL_assertion(pMesh != NULL);
		QFile file(name);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			QTextStream in(&file);
			QString line = in.readLine();
			if (line.isNull())
				return false;
			int vnum = 0;
			int fnum = 0;
			int fvnum = 0;

			while (!line.isNull())
			{
				QStringList list = line.split(" ");
				if (list.size()>2)
				{
					if (!list.at(0).compare("v"))
						vnum++;
					if (!list.at(0).compare("f"))
					{
						fnum++;
						if (fvnum == 0)
							fvnum = list.size() - 1;
					}
				}
				line = in.readLine();
			}

			in.seek(0);
			Point_3 *vertex = new Point_3[vnum];
			//Point_3 *color = new Point_3[vnum];
			int **face = new int*[fnum];
			//domain = new Point_2[vnum];

			for (int i = 0; i<fnum; i++)
			{
				face[i] = new int[fvnum];
			}
			line = in.readLine();
			int vi = 0;
			//int di = 0;
			int fi = 0;
			while (!line.isNull())
			{
				QStringList list = line.split(" ");
				if (!list.at(0).compare("v"))
				{
					vertex[vi] = Point_3(list.at(1).toDouble(), list.at(2).toDouble(), list.at(3).toDouble());
					vi++;
				}
				//else if (!list.at(0).compare("vt"))//参数化
				//{
				//	domain[di] = Point_2(list.at(1).toDouble(), list.at(2).toDouble());
				//	di++;
				//}
				else if (!list.at(0).compare("f"))
				{
					for (int i = 0; i < fvnum; i++)
					{
						face[fi][i] = list.at(i + 1).split("/").at(0).toInt() - 1;
					}
						

					/*face[fi][0] = list.at(1).split("/").at(0).toInt()-1;
					face[fi][1] = list.at(2).split("/").at(0).toInt()-1;
					face[fi][2] = list.at(3).split("/").at(0).toInt()-1;*/
					fi++;
				}
				line = in.readLine();
			}
			Builder_obj<HalfedgeDS> builder(vertex, face, vnum, fnum, fvnum);
			pMesh->delegate(builder);

			//Mesh::Vertex_iterator v_it;
			/*int v_i = 0;
			for (v_it = pMesh->vertices_begin(); v_it != pMesh->vertices_end();v_it++)
			{
			double colorv[3] = { color[v_i].x(), color[v_i].y(), color[v_i].z() };
			v_it->set_color(colorv);
			v_i++;
			}*/
			file.close();
			delete[]vertex;
			for (int i = 0; i<fnum; i++)
			{
				delete[]face[i];
			}
			delete[]face;
			/*if (di == 0)
			{
			hasParametrization = false;
			delete[]domain;
			domain = NULL;
			}
			else
			{
			hasParametrization = true;
			}*/
		}
		return true;
	}

	//bool read_withcurvature(QString &name,
	//	Enriched_polyhedron<kernel, items> *pMesh)
	//{
	//	CGAL_assertion(pMesh != NULL);
	//	QFile file(name);
	//	if (file.open(QIODevice::ReadOnly | QIODevice::Text))
	//	{
	//		QTextStream in(&file);
	//		QString line = in.readLine();
	//		if (line.isNull())
	//			return false;
	//		int vnum = 0;
	//		int fnum = 0;
	//		int fvnum = 0;

	//		while (!line.isNull())
	//		{
	//			QStringList list = line.split(" ");
	//			if (list.size()>2)
	//			{
	//				if (!list.at(0).compare("v"))
	//					vnum++;
	//				if (!list.at(0).compare("f"))
	//				{
	//					fnum++;
	//					if (fvnum == 0)
	//						fvnum = list.size() - 1;
	//				}
	//			}
	//			line = in.readLine();
	//		}

	//		in.seek(0);
	//		Point_3 *vertex = new Point_3[vnum];
	//		Point_2 *curvature = new Point_2[vnum];
	//		int **face = new int*[fnum];

	//		for (int i = 0; i<fnum; i++)
	//		{
	//			face[i] = new int[fvnum];
	//		}
	//		line = in.readLine();
	//		int vi = 0;
	//		int fi = 0;
	//		while (!line.isNull())
	//		{
	//			QStringList list = line.split(" ");
	//			if (!list.at(0).compare("v"))
	//			{
	//				vertex[vi] = Point_3(list.at(1).toDouble(), list.at(2).toDouble(), list.at(3).toDouble());
	//				curvature[vi] = Point_2(list.at(4).toDouble(), list.at(5).toDouble());
	//				vi++;
	//			}
	//			else if (!list.at(0).compare("f"))
	//			{
	//				for (int i = 0; i<fvnum; i++)
	//					face[fi][i] = list.at(i + 1).split("/").at(0).toInt() - 1;

	//				fi++;
	//			}
	//			line = in.readLine();
	//		}
	//		Builder_obj<HalfedgeDS> builder(vertex, face, vnum, fnum, fvnum);
	//		pMesh->delegate(builder);

	//		Mesh::Vertex_iterator v_it;
	//		int v_i = 0;
	//		for (v_it = pMesh->vertices_begin(); v_it != pMesh->vertices_end(); v_it++)
	//		{
	//			v_it->gaussian_curvature() = curvature[v_i].x();
	//			v_it->mean_curvature() = curvature[v_i].y();
	//			v_i++;
	//		}
	//		file.close();
	//		delete[]vertex;
	//		delete[]curvature;
	//		for (int i = 0; i<fnum; i++)
	//		{
	//			delete[]face[i];
	//		}
	//		delete[]face;
	//	}
	//	return true;
	//}

	bool read_surface_with_curvature_domain_color(QString &name,
		Enriched_polyhedron<kernel, items> *pMesh)
	{
		CGAL_assertion(pMesh != NULL);
		QFile file(name);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			QTextStream in(&file);
			QString line = in.readLine();
			if (line.isNull())
				return false;
			int vnum = 0;
			int fnum = 0;
			int fvnum = 0;

			while (!line.isNull())
			{
				QStringList list = line.split(" ");
				if (list.size() > 2)
				{
					if (!list.at(0).compare("v"))
						vnum++;
					if (!list.at(0).compare("f"))
					{
						fnum++;
						if (fvnum == 0)
							fvnum = list.size() - 1;
					}
				}
				line = in.readLine();
			}

			in.seek(0);
			Point_3 *vertex = new Point_3[vnum];
			double *mean_curvature = new double[vnum];
			double *gauss_curvature = new double[vnum];
			Point_3 *color = new Point_3[vnum];
			Point_2 *domain = new Point_2[vnum];
			int **face = new int*[fnum];

			for (int i = 0; i < fnum; i++)
			{
				face[i] = new int[fvnum];
			}
			line = in.readLine();
			int vi = 0;
			int mi = 0;
			int gi = 0;
			int ci = 0;
			int di = 0;
			int fi = 0;
			while (!line.isNull())
			{
				QStringList list = line.split(" ");
				if (!list.at(0).compare("v"))
				{
					vertex[vi] = Point_3(list.at(1).toDouble(), list.at(2).toDouble(), list.at(3).toDouble());
					vi++;
				}
				else if (!list.at(0).compare("m"))
				{
					mean_curvature[mi] = list.at(1).toDouble();
					mi++;
				}
				else if (!list.at(0).compare("g"))
				{
					gauss_curvature[gi] = list.at(1).toDouble();
					gi++;
				}
				else if (!list.at(0).compare("c"))
				{
					color[ci] = Point_3(list.at(1).toDouble(), list.at(2).toDouble(), list.at(3).toDouble());
					ci++;
				}
				else if (!list.at(0).compare("d"))
				{
					domain[di] = Point_2(list.at(1).toDouble(), list.at(2).toDouble());
					di++;
				}
				else if (!list.at(0).compare("f"))
				{
					for (int i = 0; i < fvnum; i++)
						face[fi][i] = list.at(i + 1).split("/").at(0).toInt() - 1;

					fi++;
				}
				line = in.readLine();
			}
			Builder_obj<HalfedgeDS> builder(vertex, face, vnum, fnum, fvnum);
			pMesh->delegate(builder);

			Mesh::Vertex_iterator v_it;
			int v_i = 0;
			for (v_it = pMesh->vertices_begin(); v_it != pMesh->vertices_end(); v_it++)
			{
				if (gi == vi)
				{
					v_it->gaussian_curvature() = gauss_curvature[v_i];
				}
				if (mi == vi)
				{
					v_it->mean_curvature() = mean_curvature[v_i];
				}
				if (ci == vi)
				{
					v_it->vertex_color() = color[v_i];
				}
				if (di == vi)
				{
					v_it->get_domain() = domain[v_i];
				}
				v_i++;
			}
			file.close();
			delete[]vertex;
			delete[]mean_curvature;
			delete[]gauss_curvature;
			delete[]color;
			delete[]domain;
			for (int i = 0; i < fnum; i++)
			{
				delete[]face[i];
			}
			delete[]face;
		}
		return true;
	}

	bool read_withcolor(QString &name,
		Enriched_polyhedron<kernel, items> *pMesh)
	{
		CGAL_assertion(pMesh != NULL);
		QFile file(name);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			QTextStream in(&file);
			QString line = in.readLine();
			if (line.isNull())
				return false;
			int vnum = 0;
			int fnum = 0;
			int fvnum = 0;

			while (!line.isNull())
			{
				QStringList list = line.split(" ");
				if (list.size()>2)
				{
					if (!list.at(0).compare("v"))
						vnum++;
					if (!list.at(0).compare("f"))
					{
						fnum++;
						if (fvnum == 0)
							fvnum = list.size() - 1;
					}
				}
				line = in.readLine();
			}

			in.seek(0);
			Point_3 *vertex = new Point_3[vnum];
			Point_3 *color = new Point_3[vnum];
			int **face = new int*[fnum];
			//domain = new Point_2[vnum];

			for (int i = 0; i<fnum; i++)
			{
				face[i] = new int[fvnum];
			}
			line = in.readLine();
			int vi = 0;
			//int di = 0;
			int fi = 0;
			while (!line.isNull())
			{
				QStringList list = line.split(" ");
				if (!list.at(0).compare("v"))
				{
					vertex[vi] = Point_3(list.at(1).toDouble(), list.at(2).toDouble(), list.at(3).toDouble());
					color[vi] = Point_3(list.at(4).toDouble(), list.at(5).toDouble(), list.at(6).toDouble());
					vi++;
				}
				//else if (!list.at(0).compare("vt"))//参数化
				//{
				//	domain[di] = Point_2(list.at(1).toDouble(), list.at(2).toDouble());
				//	di++;
				//}
				else if (!list.at(0).compare("f"))
				{
					for (int i = 0; i<fvnum; i++)
						face[fi][i] = list.at(i + 1).split("/").at(0).toInt() - 1;

					/*face[fi][0] = list.at(1).split("/").at(0).toInt()-1;
					face[fi][1] = list.at(2).split("/").at(0).toInt()-1;
					face[fi][2] = list.at(3).split("/").at(0).toInt()-1;*/
					fi++;
				}
				line = in.readLine();
			}
			Builder_obj<HalfedgeDS> builder(vertex, face, vnum, fnum, fvnum);
			pMesh->delegate(builder);

			Mesh::Vertex_iterator v_it;
			int v_i = 0;
			for (v_it = pMesh->vertices_begin(); v_it != pMesh->vertices_end(); v_it++)
			{
				double colorv[3] = { color[v_i].x(), color[v_i].y(), color[v_i].z()};
				v_it->set_color(colorv);
				v_i++;
			}
			file.close();
			delete[]vertex;
			delete[]color;
			for (int i = 0; i<fnum; i++)
			{
				delete[]face[i];
			}
			delete[]face;
			/*if (di == 0)
			{
			hasParametrization = false;
			delete[]domain;
			domain = NULL;
			}
			else
			{
			hasParametrization = true;
			}*/
		}
		return true;
	}

};


#endif

