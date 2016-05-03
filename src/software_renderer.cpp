#include "software_renderer.h"

#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>

#include "triangulation.h"

using namespace std;

namespace CMU462 {


// Implements SoftwareRenderer //

void SoftwareRendererImp::draw_svg( SVG& svg ) {

  // set top level transformation
  transformation = canvas_to_screen;

  // draw all elements
  for ( size_t i = 0; i < svg.elements.size(); ++i ) {
    draw_element(svg.elements[i]);
  }

  // draw canvas outline
  Vector2D a = transform(Vector2D(    0    ,     0    )); a.x--; a.y++;
  Vector2D b = transform(Vector2D(svg.width,     0    )); b.x++; b.y++;
  Vector2D c = transform(Vector2D(    0    ,svg.height)); c.x--; c.y--;
  Vector2D d = transform(Vector2D(svg.width,svg.height)); d.x++; d.y--;

  rasterize_line(a.x, a.y, b.x, b.y, Color::Black);
  rasterize_line(a.x, a.y, c.x, c.y, Color::Black);
  rasterize_line(d.x, d.y, b.x, b.y, Color::Black);
  rasterize_line(d.x, d.y, c.x, c.y, Color::Black);

  // resolve and send to render target
  resolve();

}

void SoftwareRendererImp::set_sample_rate( size_t sample_rate ) {

  // Task 3: 
  // You may want to modify this for supersampling support
  this->sample_rate = sample_rate;

}

void SoftwareRendererImp::set_render_target( unsigned char* render_target,
                                             size_t width, size_t height ) {

  // Task 3: 
  // You may want to modify this for supersampling support
  this->render_target = render_target;
  this->target_w = width;
  this->target_h = height;
  this->supersample_w = width * this->sample_rate;
  this->supersample_h = height * this-> sample_rate;
  this->supersample_target = (unsigned char*)malloc(supersample_w * supersample_h * sizeof(char));
}

void SoftwareRendererImp::draw_element( SVGElement* element ) {

  // Task 4 (part 1):
  // Modify this to implement the transformation stack

  switch(element->type) {
    case POINT:
      draw_point(static_cast<Point&>(*element));
      break;
    case LINE:
      draw_line(static_cast<Line&>(*element));
      break;
  case POLYLINE:
      draw_polyline(static_cast<Polyline&>(*element));
      break;
    case RECT:
      draw_rect(static_cast<Rect&>(*element));
      break;
    case POLYGON:
      draw_polygon(static_cast<Polygon&>(*element));
      break;
    case ELLIPSE:
      draw_ellipse(static_cast<Ellipse&>(*element));
      break;
    case IMAGE:
      draw_image(static_cast<Image&>(*element));
      break;
    case GROUP:
      draw_group(static_cast<Group&>(*element));
      break;
    default:
      break;
  }

}


// Primitive Drawing //

void SoftwareRendererImp::draw_point( Point& point ) {

  Vector2D p = transform(point.position);
  rasterize_point( p.x, p.y, point.style.fillColor );

}

void SoftwareRendererImp::draw_line( Line& line ) { 

  Vector2D p0 = transform(line.from);
  Vector2D p1 = transform(line.to);
  rasterize_line( p0.x, p0.y, p1.x, p1.y, line.style.strokeColor );

}

void SoftwareRendererImp::draw_polyline( Polyline& polyline ) {

  Color c = polyline.style.strokeColor;

  if( c.a != 0 ) {
    int nPoints = polyline.points.size();
    for( int i = 0; i < nPoints - 1; i++ ) {
      Vector2D p0 = transform(polyline.points[(i+0) % nPoints]);
      Vector2D p1 = transform(polyline.points[(i+1) % nPoints]);
      rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    }
  }
}

void SoftwareRendererImp::draw_rect( Rect& rect ) {

  Color c;
  
  // draw as two triangles
  float x = rect.position.x;
  float y = rect.position.y;
  float w = rect.dimension.x;
  float h = rect.dimension.y;

  Vector2D p0 = transform(Vector2D(   x   ,   y   ));
  Vector2D p1 = transform(Vector2D( x + w ,   y   ));
  Vector2D p2 = transform(Vector2D(   x   , y + h ));
  Vector2D p3 = transform(Vector2D( x + w , y + h ));
  
  // draw fill
  c = rect.style.fillColor;
  if (c.a != 0 ) {
    rasterize_triangle( p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, c );
    rasterize_triangle( p2.x, p2.y, p1.x, p1.y, p3.x, p3.y, c );
  }

  // draw outline
  c = rect.style.strokeColor;
  if( c.a != 0 ) {
    rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    rasterize_line( p1.x, p1.y, p3.x, p3.y, c );
    rasterize_line( p3.x, p3.y, p2.x, p2.y, c );
    rasterize_line( p2.x, p2.y, p0.x, p0.y, c );
  }

}

void SoftwareRendererImp::draw_polygon( Polygon& polygon ) {

  Color c;

  // draw fill
  c = polygon.style.fillColor;
  if( c.a != 0 ) {

    // triangulate
    vector<Vector2D> triangles;
    triangulate( polygon, triangles );

    // draw as triangles
    for (size_t i = 0; i < triangles.size(); i += 3) {
      Vector2D p0 = transform(triangles[i + 0]);
      Vector2D p1 = transform(triangles[i + 1]);
      Vector2D p2 = transform(triangles[i + 2]);
      rasterize_triangle( p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, c );
    }
  }

  // draw outline
  c = polygon.style.strokeColor;
  if( c.a != 0 ) {
    int nPoints = polygon.points.size();
    for( int i = 0; i < nPoints; i++ ) {
      Vector2D p0 = transform(polygon.points[(i+0) % nPoints]);
      Vector2D p1 = transform(polygon.points[(i+1) % nPoints]);
      rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    }
  }
}

void SoftwareRendererImp::draw_ellipse( Ellipse& ellipse ) {

  // Extra credit 

}

void SoftwareRendererImp::draw_image( Image& image ) {

  Vector2D p0 = transform(image.position);
  Vector2D p1 = transform(image.position + image.dimension);

  rasterize_image( p0.x, p0.y, p1.x, p1.y, image.tex );
}

void SoftwareRendererImp::draw_group( Group& group ) {

  for ( size_t i = 0; i < group.elements.size(); ++i ) {
    draw_element(group.elements[i]);
  }

}

// Rasterization //

// The input arguments in the rasterization functions 
// below are all defined in screen space coordinates

void SoftwareRendererImp::rasterize_point( float x, float y, Color color ) {

  // fill in the nearest pixel
  int sx = (int) floor(x);
  int sy = (int) floor(y);

  // check bounds
  if ( sx < 0 || sx >= target_w ) return;
  if ( sy < 0 || sy >= target_h ) return;

  // fill sample - NOT doing alpha blending!
  render_target[4 * (sx + sy * target_w)    ] = (uint8_t) (color.r * 255);
  render_target[4 * (sx + sy * target_w) + 1] = (uint8_t) (color.g * 255);
  render_target[4 * (sx + sy * target_w) + 2] = (uint8_t) (color.b * 255);
  render_target[4 * (sx + sy * target_w) + 3] = (uint8_t) (color.a * 255);

}

  /* Implementación del algoritmo de Bresenham */
void SoftwareRendererImp::rasterize_line( float x0, float y0,
                                          float x1, float y1,
                                          Color color) {
  double x, y, dx, dy, p, incre, inc, stepx, stepy; /* Constantes necesarias para el algoritmo (algoritmo sacado de https://es.wikipedia.org/wiki/Algoritmo_de_Bresenham#Algoritmo) */
  dx = (x1-x0);
  dy = (y1-y0);
  if(dy < 0){
    dy = -dy;
    stepy = -1;
  }else
    stepy = 1;
  if(dx < 0){
    dx = -dx;
    stepx = -1;
  }
  else
    stepx = 1;
  x = x0;
  y = y0;
  rasterize_point(x0, y0, color);
  if(dx > dy){
    p = 2*dy - dx;
    incre = 2*dy;
    inc = 2*(dy-dx);
    while(abs(x - x1) >= 1){
      x += stepx;
      if(p < 0)
	p += incre;
      else{
	y += stepy;
	p += inc;
      }
      rasterize_point(x, y, color);
    }
  }else{
    p = 2*dx - dy;
    incre = 2*dx;
    inc = 2*(dx-dy);
    while(abs(y - y1) >= 1){
      y += stepy;
      if(p < 0)
	p += incre;
      else{
	x += stepx;
	p += inc;
      }
      rasterize_point(x, y, color);
    }
  }
}
  /* Nuestro rasterize_triangle se divide a nuestro triángulo en dos partes, uno derecho y uno volteado. Haremos un método para pintar a cada uno */
  
  /* Pinta un triágunlo derecho (con base horizontal). Supone de antemano que y1 = y2 */
  void SoftwareRendererImp::pinta_triangulo_derecho(float x0, float y0,
						    float x1, float y1,
						    float x2, float y2,
						    Color color){
    float pendiente1 = (x1-x0) / (y1-y0); /* La pendiente de la recta entre el punto v0 y v1 */
    float pendiente2 = (x2-x0) / (y2-y0); /* La pendiente de la recta entre el punto v0 y v2 */
    float xactual1 = x0;
    float xactual2 = x0; /* La coordenada x del punto actual */
    /* En este for se van dibujando líneas horizontales */
    for (int linea = y0; linea <= y1; linea++){
      rasterize_line(xactual1, linea, xactual2, linea, color);
      xactual1 += pendiente1;
      xactual2 += pendiente2;
    }
  }

  /* Pinta un triágunlo volteado(uno derecho girado 180°). Supone de antemano que y0 = y1 */
  void SoftwareRendererImp::pinta_triangulo_volteado(float x0, float y0,
						    float x1, float y1,
						    float x2, float y2,
						    Color color){
    float pendiente1 = (x2-x0) / (y2-y0); /* La pendiente de la recta entre el punto v0 y v1 */
    float pendiente2 = (x2-x1) / (y2-y1); /* La pendiente de la recta entre el punto v0 y v2 */
    float xactual1 = x2; 
    float xactual2 = x2; /* La coordenada x del punto actual */
    /* En este for se van dibujando líneas horizontales (ahora es hacia arriba)*/
    for (int linea = y2; linea > y0; linea--){
      xactual1 -= pendiente1;
      xactual2 -= pendiente2;
      rasterize_line(xactual1, linea, xactual2, linea, color);
    }
  }



  /* Utiliza el algoritmo regular de rasterización de triángulos. Sacado de http://www.sunshine2k.de/coding/java/TriangleRasterization/TriangleRasterization.html */
void SoftwareRendererImp::rasterize_triangle( float x0, float y0,
                                              float x1, float y1,
                                              float x2, float y2,
                                              Color color ) {
  /* Primero ordenamos las coordenadas por y */
  float ymax, ysnd, ythrd, xmax, xsnd, xthrd; /* La y más grande, la de en medio y la más pequeña (lo mismo para x) */
  if(y0<y1){ //c1-1
    if(y1>y2){ //c2-1
      if(y0>y2){ //c3-1
	ymax = y1;
	xmax = x1;
	ysnd = y0;
	xsnd = x0;
	ythrd = y2;
	xthrd = x2;
      }else{ //c3-2
	ymax = y1;
	xmax = x1;
	ysnd = y2;
	xsnd = x2;
	ythrd = y0;
	xthrd = x0;
      }
    }else{ //c2-2
      ymax = y2;
      xmax = x2;
      ysnd = y1;
      xsnd = x1;
      ythrd = y0;
      xthrd = x0;
    } 
  }else{ //c1-1
    if(y0>y2){ //c4-1
      if(y2>y1){ //c5-1
	ymax = y0;
	xmax = x0;
	ysnd = y2;
	xsnd = x2;
	ythrd = y1;
	xthrd = x1;
      }else{ //c5-2
	ymax = y0;
	xmax = x0;
	ysnd = y1;
	xsnd = x1;
	ythrd = y2;
	xthrd = x2;
      }
    }else{ //c4-2
      ymax = y2;
      xmax = x2;
      ysnd = y0;
      xsnd = x0;
      ythrd = y1;
      xthrd = x1;
    }
  } 
  
  /* Si ymax == ysnd, tenemos un triángulo derecho */
  if(abs(ymax - ysnd) < 1)
    pinta_triangulo_derecho(xthrd, ythrd, xsnd, ysnd, xmax, ymax, color);
  /* Si ythrd == ysnd, tenemos un triángulo volteado */
  else if(abs(ysnd - ythrd) < 1)
    pinta_triangulo_volteado(xthrd, ythrd, xsnd, ysnd, xmax, ymax, color);
  else{
    /* Caso general */
    float x4 = xthrd + (((ysnd - ythrd) / (ymax - ythrd)) * (xmax - xthrd)); /* Coordenada x en la cual se parte el triángulo */
    float y4 = ysnd; /* Coordenada y en la cual se parte el triángulo */
    pinta_triangulo_derecho(xthrd, ythrd, xsnd, ysnd, x4, y4, color);
    pinta_triangulo_volteado(xsnd, ysnd, x4, y4, xmax, ymax, color); 
  }
}
  
void SoftwareRendererImp::rasterize_image( float x0, float y0,
                                           float x1, float y1,
                                           Texture& tex ) {
  // Task ?: 
  // Implement image rasterization

}

// resolve samples to render target
void SoftwareRendererImp::resolve( void ) {

  // Task 3: 
  // Implement supersampling
  // You may also need to modify other functions marked with "Task 3".
  free(this->supersample_target);
  return;

}


} // namespace CMU462
