#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>

//#include "inc/pointspring.h"
//#include "inc/pointcable.h"

#include <pe/vector.h>
#include <pe/matrix.h>
#include <pe/rigidbody.h>
#include <pe/spring.h>

//#include <broadcollision.h>
#include <pe/narrowcollision.h>
#include <pe/resolution.h>

#include <ge/fb.h>
#include <ge/cam.h>
#include <ge/draw.h>

#define dTime 1/64.0
#define G 9.81

/*void printCollision(PointCollision *pC) {
  printf("coord1: %7.2f, %7.2f, %7.2f\n", pC->pPoint1->p.x, pC->pPoint1->p.y, pC->pPoint1->p.z);
  printf("coord2: %7.2f, %7.2f, %7.2f\n", pC->pPoint2->p.x, pC->pPoint2->p.y, pC->pPoint2->p.z);
  printf("normal: %7.2f, %7.2f, %7.2f\n", pC->normal.x, pC->normal.y, pC->normal.z);
  printf("penetration: %7.2f\n", pC->penetration);
}*/

Matrix33 cubeiit = {1, 0, 0, 0, 1, 0, 0, 0, 1};
Matrix34 null34 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int main() {

  Collision collisions[999];
  int collisionC = 0;

  uint32_t *b = fbInit();
  Rigidbody rb = {{10, 0, 10}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0, 0}, 1, cubeiit, null34};
  Rigidbody rb2 = {{8, 3, 10}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0.901, 0, 0, 0.434}, 1, cubeiit, null34};
  rb2.o = qNorm(rb2.o);
  rb.transformMatrix = m34FromQV(rb.o, rb.p);
  rb2.transformMatrix = m34FromQV(rb2.o, rb2.p);
  CollisionBox cb = {&rb, {0, 0, 0}, {2, 2, 2}};
  CollisionBox cb2 = {&rb2, {0, 0, 0}, {2, 2, 2}};

  Camera cam = {{0, 0, 0}, 100, {1, 0, 0, 0}, null34};
  cam.transform = m34FromQV(cam.o, cam.p);
  
  for (;;) {
    BoxBoxCollision(&cb, &cb2, &collisions[collisionC]);
    collisionC++;
    for (int i = 0; i < collisionC; i++)
      resolveCollision(&collisions[i]);
    collisionC = 0;

    drawBox(b, &cam, &cb, red);
    drawBox(b, &cam, &cb2, blue);
    usleep((int)(1000000*dTime));
    drawBox(b, &cam, &cb, black);
    drawBox(b, &cam, &cb2, black);
  }
  
  /*for (int i = 0; i < 3; i++) {
    printf("pen: %f ", collisions[i].penetration);
    printV(collisions[i].p);
  }*/
}

/*int main() {

  fbInit();

  //pointCollisions = (PointCollision*)malloc(sizeof(PointCollision) * 99);

  Matrix33 cubeiit = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  Matrix34 null34 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  Rigidbody rb = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, -G, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0, 0}, 1, cubeiit, null34};
  Rigidbody rb2 = {{10, 10, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0, 0}, 0, cubeiit, null34};
  RigidbodySpring s = {{2, 2, 0}, {2, 2, 0}, &rb, &rb2, 1, 10};
  for (int i = 0; i < 64*10000; i++) {
    rb.transformMatrix = m34FromQV(rb.o, rb.p);
    rb2.transformMatrix = m34FromQV(rb2.o, rb2.p);
    updateRigidbodySpringForces(&s);
    updateRigidbody(&rb, dTime);
    updateRigidbody(&rb2, dTime);

    Vector rbp = m34vMult(rb.transformMatrix, s.p1);
    Vector rbp2 = m34vMult(rb2.transformMatrix, s.p2);
    drawV(rb.p, red);
    drawV(rb2.p, yellow);
    drawV(rbp, green);
    drawV(rbp2, blue);
    usleep((int)(1000000*dTime));
    drawV(rb.p, black);
    drawV(rb2.p, black);
    drawV(rbp, black);
    drawV(rbp2, black);
  }
}*/
