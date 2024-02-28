#include "student_code.h"
#include "mutablePriorityQueue.h"
#include "halfEdgeMesh.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    std::vector<Vector2D> nextLevel;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
        Vector2D interpolatedPoint = (1 - t) * points[i] + t * points[i + 1];
        nextLevel.push_back(interpolatedPoint);
    }
    return nextLevel;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      std::vector<Vector3D> nextLevel;
      for (size_t i = 0; i < points.size() - 1; i++)
      {
          Vector3D interpolatedPoint = (1 - t) * points[i] + t * points[i + 1];
          nextLevel.push_back(interpolatedPoint);
      }
      return nextLevel;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
   if (points.size() == 1) {
          return points[0];
   }
  else {
       std::vector<Vector3D> nextLevel = evaluateStep(points, t);
       return evaluate1D(nextLevel, t);
   }
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
     std::vector<Vector3D> curvePoints;
     for (const auto& row : controlPoints) {
         curvePoints.push_back(evaluate1D(row, u));
     }
     return evaluate1D(curvePoints, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    Vector3D n(0, 0, 0); 
    HalfedgeCIter h = this->halfedge();
    h = h->twin();
    HalfedgeCIter h0 = h;
    n += h->face()->normal();
    h = h->next()->twin();
    while (h != h0 && !(h->isBoundary())) {
        n += h->face()->normal();
        h = h->next()->twin();
    }
    return n.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
      if (e0->isBoundary()) return e0;
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();
      h0->setNeighbors(h5, h3, v2, e0, f0);
      h3->setNeighbors(h2, h0, v3, e0, f1);
      h5->setNeighbors(h1, h5->twin(), v3, h5->edge(),f0);
      h1->setNeighbors(h0, h1->twin(), v1, h5->edge(), f0);
      h2->setNeighbors(h4, h2->twin(), v2, h2->edge(), f1);
      h4->setNeighbors(h3, h4->twin(), v0, h4->edge(), f1);
      v0->halfedge() = h4;
      v2->halfedge() = h0;
      v1->halfedge() = h1;
      v3->halfedge() = h3;
      f0->halfedge() = h0;
      f1->halfedge() = h3;
      return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      if (e0->isBoundary())
      {
          HalfedgeIter h0 = e0->halfedge();
          HalfedgeIter h1 = h0->next();
          HalfedgeIter h2 = h1->next();
          VertexIter v0 = h0->vertex();
          VertexIter v1 = h1->vertex();
          VertexIter v2 = h2->vertex();
          VertexIter m = newVertex();
          FaceIter f0 = h0->face();
          m->position = 0.5 * (v0->position + v1->position);
          HalfedgeIter hh0 = newHalfedge();
          HalfedgeIter hh1 = newHalfedge();
          HalfedgeIter hh2 = newHalfedge();
          EdgeIter e1 = newEdge();
          EdgeIter e2 = newEdge();
          FaceIter f1 = newFace();
          m->halfedge() = hh0;
          h0->setNeighbors(hh2, h0->twin(), v0, e0, f0);
          hh0->setNeighbors(h1, h0->twin(), m, e1, f1);
          h2->setNeighbors(h0, h2->twin(), v2, h2->edge(), f0);
          h1->setNeighbors(hh1, h1->twin(), v1, h1->edge(), f1);
          hh2->setNeighbors(h2, hh1, m, e1, f0);
          hh1->setNeighbors(hh0, hh2, v2, e1, f1);
          f1->halfedge() = hh0;
          e1->halfedge() = hh0;
          e2->halfedge() = hh1;
          e1->isNew = true;
          e2->isNew = true;

          return m;
      }
          

      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();
      HalfedgeIter hh0 = newHalfedge();
      HalfedgeIter hh1 = newHalfedge();
      HalfedgeIter hh2 = newHalfedge();
      HalfedgeIter hh3 = newHalfedge();
      HalfedgeIter hh4 = newHalfedge();
      HalfedgeIter hh5 = newHalfedge();
      EdgeIter e1 = newEdge();
      EdgeIter e2 = newEdge();
      EdgeIter e3 = newEdge();
      FaceIter f2 = newFace();
      FaceIter f3 = newFace();
      VertexIter m = newVertex();
      m->position = 0.5 * (v0->position + v1->position);
      m->halfedge() = hh0;
      h0->setNeighbors(hh2, hh3, v0, e0, f0);
      h3->setNeighbors(hh5, hh0, v1, e1, f3);
      h5->setNeighbors(h3, h5->twin(), v3, h5->edge(),f3);
      h1->setNeighbors(hh1, h1->twin(), v1, h1->edge(), f1);
      h2->setNeighbors(h0, h2->twin(), v2, h2->edge(), f0);
      h4->setNeighbors(hh4, h4->twin(), v0, h4->edge(), f2);
      hh0->setNeighbors(h1, h3, m, e1, f1);
      hh1->setNeighbors(hh0, hh2, v2, e2, f1);
      hh2->setNeighbors(h2, hh1, m, e2, f0);
      hh3->setNeighbors(h4, h0, m, e0, f2);
      hh4->setNeighbors(hh3, hh5, v3, e3, f2);
      hh5->setNeighbors(h5, hh4, m, e3, f3);
      f0->halfedge() = h0;
      f1->halfedge() = hh0;
      f2->halfedge() = hh3;
      f3->halfedge() = h3;
      e1->halfedge() = hh0;
      e2->halfedge() = hh1;
      e3->halfedge() = hh5;
      e0->isNew = false;
      e1->isNew = false;
      e2->isNew = true;
      e3->isNew = true;
      return m;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.
      for (VertexIter v = mesh.verticesBegin();v != mesh.verticesEnd();v++)
      {
          float n = v->degree();
          float u = 3.0 / 8.0 / n;
          if (n == 3) u = 3.0 / 16.0;
          v->isNew = false;
          Vector3D neighbours(0, 0, 0);
          HalfedgeCIter h = v->halfedge();
          h = h->twin();
          neighbours += h->vertex()->position;
          h = h->next();
          while (h != v->halfedge())
          {
              h = h->twin();
              neighbours += h->vertex()->position;
              h = h->next();
          }
          v->newPosition = (1 - n * u) * v->position + u * neighbours;
      }
      for (EdgeIter e = mesh.edgesBegin();e != mesh.edgesEnd(); e++)
      {
          Vector3D v0 = e->halfedge()->vertex()->position;
          Vector3D v1 = e->halfedge()->twin()->vertex()->position;
          Vector3D v2 = e->halfedge()->next()->next()->vertex()->position;
          Vector3D v3 = e->halfedge()->twin()->next()->next()->vertex()->position;

          e->newPosition = 3.0 / 8.0 * (v0 + v1) + 1.0 / 8.0 * (v2 + v3);
          e->isNew = false;
      }
      int num = mesh.nEdges();
      int i = 0;
      for (EdgeIter e = mesh.edgesBegin();i < num;i++)
      {
          if (e->isNew == false)
          {
              VertexIter vv = mesh.splitEdge(e);
              vv->position = e->newPosition;
              vv->isNew = true;
          }
          e++;
      }
      for (EdgeIter e = mesh.edgesBegin();e != mesh.edgesEnd(); e++)
      {
          if (e->isNew)
          {
              HalfedgeIter hh = e->halfedge();
              if ((hh->vertex()->isNew && !(hh->twin()->vertex()->isNew)) || (!(hh->vertex()->isNew) && hh->twin()->vertex()->isNew))
              {
                  e = mesh.flipEdge(e);
                  e->isNew = false;
              }
          }
      }
      for (VertexIter v = mesh.verticesBegin();v != mesh.verticesEnd();v++) 
      {
          if (v->isNew == false)
              v->position = v->newPosition;
          else
              v->isNew = false;
      }
  }
}
