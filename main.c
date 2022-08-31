#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>
#include <pthread.h>

#include <pe/vector.h>
#include <pe/matrix.h>
#include <pe/rigidbody.h>
#include <pe/spring.h>

//#include <broadcollision.h>
#include <pe/collision.h>
#include <pe/narrowcollision.h>
#include <pe/resolution.h>

#include <ge/fb.h>
#include <ge/cam.h>
#include <ge/draw.h>
#include <ge/wl.h>
#include <ge/seat.h>

#define dTime 1/64.0
#define G 9.81

int slow = 1;

bool stop = 0;

/*void printCollision(PointCollision *pC) {
  printf("coord1: %7.2f, %7.2f, %7.2f\n", pC->pPoint1->p.x, pC->pPoint1->p.y, pC->pPoint1->p.z);
  printf("coord2: %7.2f, %7.2f, %7.2f\n", pC->pPoint2->p.x, pC->pPoint2->p.y, pC->pPoint2->p.z);
  printf("normal: %7.2f, %7.2f, %7.2f\n", pC->normal.x, pC->normal.y, pC->normal.z);
  printf("penetration: %7.2f\n", pC->penetration);
}*/

void printCollision(Collision *pC) {
  printf("p: %7.2f, %7.2f, %7.2f\n", pC->p.x, pC->p.y, pC->p.z);
  printf("normal: %7.2f, %7.2f, %7.2f\n", pC->normal.x, pC->normal.y, pC->normal.z);
  printf("penetration: %7.2f\n", pC->penetration);
}

Collision collisions[999];
int collisionC = 0;

void drawCollision(uint32_t *b, Camera *cam, Collision *pC, uint32_t color) {
  //if (!collisionC) return;
  drawLine(b, worldToScreen(cam, pC->p), worldToScreen(cam, vAdd(pC->p, vMult(pC->normal, 3))), color);
  drawLine(b, worldToScreen(cam, vAdd(pC->p, vMult(pC->normal, 3))), worldToScreen(cam, vAdd(pC->p, vMult(pC->normal, 4))), color + 0x00ff0000);
}

Matrix33 cubeiit = {6.0/16.0, 0, 0, 0, 6.0/16.0, 0, 0, 0, 6.0/16.0};
Matrix34 null34 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

Camera cam;
CollisionBox cb;
CollisionBox cb2;
CollisionBox cb3;

void key(uint32_t key, uint8_t state) {
  if (state == 1) return;
  switch (key) {
    case 25:
      cam.p.y+=2;
      break;
    case 38:
      cam.p.x-=2;
      break;
    case 39:
      cam.p.y-=2;
      break;
    case 40:
      cam.p.x+=2;
      break;
    case 65:
      stop = !stop;
  }
  cam.transform = m34FromQV(cam.o, cam.p);
  //printf("key: %d\n", key);
  //printV(cam.p);
}

void *wl() {
  uint32_t *b = init();
  while (wl_display_dispatch(wl_display)) {
    for (int i = 0; i < 1080; i++)
      for (int j = 0; j < 1920; j++)
        b[i*1920+j] = 0;
    drawBox(b, &cam, &cb, red);
    drawBox(b, &cam, &cb2, blue);
    drawBox(b, &cam, &cb3, white);
    drawCollision(b, &cam, &collisions[0], green);
    drawCollision(b, &cam, &collisions[1], green);
    drawCollision(b, &cam, &collisions[2], green);
  }
  end();
  return NULL;
}

int main() {
  pthread_t tid;
  pthread_create(&tid, NULL, wl, NULL);

  //uint32_t *b = fbInit();
  //Rigidbody rb = {{2, 10, 0}, {0, 0, 0}, {0, 0, 0}, {0, -G, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0.01, 0.01, 0.01}, 1, cubeiit, null34};
  //Rigidbody rb = {{3, 10, 0}, {0, 0, 0}, {0, 0, 0}, {0, -G, 0}, {0, 0, 0}, {0, 0, 0}, {0.92, 0, 0, 0.38}, 1, cubeiit, null34};
  Rigidbody rb = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0, 0}, 1, cubeiit, null34};
  Rigidbody rb2 = {{-3, 5, 0}, {0, 0, 0}, {0, 0, 0}, {0, -G, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0, 0}, 1, cubeiit, null34};
  Rigidbody rb3 = {{3, 9, 0}, {0, 0, 0}, {0, 0, 0}, {0, -G, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0, 0}, 1, cubeiit, null34};
  //rb.o = qvAdd(rb.o, (Vector){0, 0.5, 0});
  rb.o = qNorm(rb.o);
  rb2.o = qNorm(rb2.o);
  rb.transform = m34FromQV(rb.o, rb.p);
  rb2.transform = m34FromQV(rb2.o, rb2.p);
  rb3.transform = m34FromQV(rb3.o, rb3.p);
  cb = (CollisionBox){&rb, {0, 0, 0}, {2, 2, 2}};
  cb2 = (CollisionBox){&rb2, {0, 0, 0}, {2, 2, 2}};
  cb3 = (CollisionBox){&rb3, {0, 0, 0}, {2, 2, 2}};

  cam = (Camera){{-2, 2, -10}, 200, {1, 0, 0, 0}, null34};
  cam.transform = m34FromQV(cam.o, cam.p);

  ConvexPolyhedra ph = {&rb, {0, 0, 0}, {2, 2, 2}};
  ConvexPolyhedra ph2 = {&rb2, {0, 0, 0}, {2, 2, 2}};
  ConvexPolyhedra ph3 = {&rb3, {0, 0, 0}, {2, 2, 2}};

  for (;;) {
    usleep((int)(1000000*dTime*slow));
    if (stop) continue;
    updateRigidbody(&rb, dTime);
    updateRigidbody(&rb2, dTime);
    updateRigidbody(&rb3, dTime);
    rb.transform = m34FromQV(rb.o, rb.p);
    rb2.transform = m34FromQV(rb2.o, rb2.p);
    rb3.transform = m34FromQV(rb3.o, rb3.p);

    collisionC = 0;
    collisionC += collision(&ph, &ph2, &collisions[collisionC]);
    collisionC += collision(&ph, &ph3, &collisions[collisionC]);
    collisionC += collision(&ph2, &ph3, &collisions[collisionC]);
    //collisionC += BoxBoxCollision(&cb, &cb2, &collisions[collisionC]);

    for (int i = 0; i < collisionC; i++) {
      resolveInterpenetration(&collisions[i]);
      resolveVelocity(&collisions[i]);
      //printCollision(&collisions[i]);
      //printf("---------------------------------------------------------------------------\n");
    }

    //usleep((int)(1000000*dTime*slow));
  }
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
