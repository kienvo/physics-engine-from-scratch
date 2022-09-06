#include <SFML/Audio.hpp>
#include <SFML/Graphics.hpp>
#include <variant>
#include <iostream>
#include <math.h>
#include <ctime>
#include <cassert>

using namespace sf;
using namespace std;

#define K_SPRING 	100000.f // N/m
#define GRAVITY_SCALE 2.f	// For better look

struct CollisionPoints {
	Vector2f A;
	Vector2f B;
	Vector2f Normal;
	float Depth;
	bool HasCollision;
};

/* Multiple dispatch
 * http://www.eptacom.net/pubblicazioni/pub_eng/mdisp.html
 * https://stackoverflow.com/questions/1749534/multiple-dispatch-in-c
*/

class Circle;
class Ground;
class Collision;
namespace algo {
	float DotProduct(Vector2f v1, Vector2f v2) {
		return v1.x*v2.x + v1.y*v2.y;
	}
	float InnerProduct(Vector2f v1, Vector2f v2) {
		return v1.y*v2.x + v1.x*v2.y;
	}
	float distance(Vector2f p1, Vector2f p2){
		Vector2f dv = p1 - p2;
		return sqrt(dv.x*dv.x + dv.y*dv.y);
	}
	
	float distance(Vector2f p){
		return distance({0,0},p);
	}

	Vector2f normalize(Vector2f d) {
		return d/distance({0,0},d);
	}

	float cos(Vector2f a, Vector2f b) {
		return DotProduct(a,b)/(distance(a)*distance(b));
	}
	
	Vector2f symmetry(Vector2f vi, Vector2f n) {
	float cosine = algo::cos(n,vi);
		Vector2f np = n*algo::distance(vi)*cosine;
		Vector2f d = vi-np;
		Vector2f vf = 2.f*d-vi;
	}
	CollisionPoints CircleCircleCollisionPoints(const Circle* a, const Circle* b);
	
	CollisionPoints CircleGroundCollisionPoints(const Circle* a, const Ground* b);
}

struct Object {
	virtual CollisionPoints isCollided(const Object* obj) const = 0;
	virtual CollisionPoints isCollided(const Circle* obj) const = 0;
	virtual CollisionPoints isCollided(const Ground* obj) const = 0;
};


class Circle: 
public CircleShape,
public Object
{
private:
public:
	using CircleShape::CircleShape;
	CollisionPoints isCollided(const Object* obj) const override { 
		return obj->isCollided(this);
	}
	CollisionPoints isCollided(const Circle* obj)  const override {
		return algo::CircleCircleCollisionPoints(this,obj);
	}
	CollisionPoints isCollided(const Ground* obj)  const override {
		return algo::CircleGroundCollisionPoints(this, obj);
	}
	Vector2f Velocity;
	Vector2f Force;
	float Mass;
};


class Ground: 
public RectangleShape,
public Object
{
private:
	Vector2f m_line;
public:
	CollisionPoints isCollided(const Object* obj) const override {
		return obj->isCollided(this);
	}
	CollisionPoints isCollided(const Circle* obj) const override {
		return obj->isCollided(this);
	}
	CollisionPoints isCollided(const Ground* obj) const override {
		return {}; // no Ground vs Ground collision
	}
	Ground(Vector2f line) {
		setLine(line);
	}
	void setLine(Vector2f line) {
		this->setSize(Vector2f(sqrt(line.x*line.x + line.y*line.y),2));
		this->setRotation(atan(line.y/line.x)/M_PI*180.f);
		m_line = line;
	}
	Vector2f getLine() {
		return m_line;
	}
	Vector2f PerpendicularClockwise() {
		return {m_line.y, -m_line.x};
	}
	Vector2f PerpendicularCounterClockwise() const {
		return Vector2f{-m_line.y, m_line.x};
	}
};
struct Collision {
	Object* ObjA;
	Object* ObjB;
	CollisionPoints Points;
};


CollisionPoints algo::CircleCircleCollisionPoints(const Circle* a, const Circle* b) {
	CollisionPoints p;
	Vector2f atob = b->getPosition() - a->getPosition();
	Vector2f btoa = a->getPosition() - b->getPosition();
	p.Normal = normalize(atob);
	p.Depth = ((a->getRadius() + b->getRadius()) 
			- distance(a->getPosition(), b->getPosition()) )/2.0;
	p.A = a->getPosition() +(a->getRadius() - p.Depth)*normalize(atob);
	p.B = b->getPosition() +(b->getRadius() - p.Depth)*normalize(btoa);
	p.HasCollision = (p.Depth >= 0.0f);
	return p;
}

CollisionPoints algo::CircleGroundCollisionPoints(const Circle* a, const Ground* b) {
	CollisionPoints p;
	Vector2f bper = b->PerpendicularCounterClockwise();
	p.Normal = normalize(bper);
	Vector2f ran = a->getPosition() - b->getPosition();
	float CenterToLine = DotProduct(ran,bper)/distance({0,0},bper);
	p.Depth = a->getRadius() - abs(CenterToLine);
	if(CenterToLine < 0.0f) { // and Norm is the same direction
		p.A = a->getPosition() +(a->getRadius())*p.Normal;
		p.B = a->getPosition() +(a->getRadius() - p.Depth)*-p.Normal;
	} else {
		p.A = a->getPosition() +a->getRadius()*p.Normal;
		p.B = a->getPosition() +(a->getRadius() - p.Depth)*p.Normal;
	}
	p.HasCollision = (p.Depth > 0.0f);
	return p;
}

bool stop=false; // For testing
class PhysicsWorld {
private:
	vector<Object*> m_objects;
	Vector2f m_gravity = Vector2f(0, -9.18f*GRAVITY_SCALE); // 
	float distance(Vector2f p1, Vector2f p2){
		Vector2f dv = p1 - p2;
		return sqrt(dv.x*dv.x + dv.y*dv.y);
	}
	bool isCollided(Circle *a, Circle *b){
		if(distance(a->getPosition(), b->getPosition())<(a->getRadius()+b->getRadius())){
			return true;
		} else return false;
	}
	Vector2f normalize(Vector2f d) {
		return d/distance({0,0},d);
	}
	void sovleCollision(Circle* a, Circle* b, CollisionPoints p, float dt) {
			//https://en.wikipedia.org/wiki/Momentum#:~:text=In%20general%2C%20when%20the%20initial%20velocities%20are%20known%2C%20the%20final%20velocities%20are%20given%20by
			Vector2f apos = a->getPosition();
			Vector2f bpos = b->getPosition();
			Vector2f amove = p.Normal *p.Depth;
			Vector2f bmove = -p.Normal *p.Depth;

			apos += amove;
			bpos += bmove;
			// Repositioning the objects
			a->setPosition(apos);
			b->setPosition(bpos);

			// Declare the equation
			// https://en.wikipedia.org/wiki/Elastic_collision#:~:text=In%20an%20angle-free%20representation%2C%20the%20changed%20velocities%20are%20computed%20using%20the%20centers%20x1%20and%20x2%20at%20the%20time%20of%20contact%20as
			float masssum = a->Mass + b->Mass;
			float m1 = a->Mass;
			Vector2f u1 = a->Velocity;//*algo::cos(a->Velocity, -p.Normal);
			float m2 = b->Mass;
			Vector2f u2 = b->Velocity;//*algo::cos(b->Velocity, p.Normal);
			Vector2f x1 = apos;
			Vector2f x2 = bpos;
			float x1subx2 = (algo::distance(x1-x2));
			float x2subx1 = (algo::distance(x2-x1));

			Vector2f v1 = u1 - (2*m2/masssum)*(algo::DotProduct(u1-u2, x1-x2)/(x1subx2*x1subx2))*(x1-x2);
			Vector2f v2 = u2 - (2*m1/masssum)*(algo::DotProduct(u2-u1, x2-x1)/(x2subx1*x2subx1))*(x2-x1);

			// TODO: Add  coefficient of restitution to objects properties
			// Energy loss,  coefficient of restitution
			a->Velocity = v1;//*0.9f;
			b->Velocity = v2;//*0.9f;
	}
	void sovleCollision(Circle* a, Ground* b, CollisionPoints p, float dt) {
			Vector2f apos = a->getPosition();
			Vector2f amove = p.Normal *p.Depth*2.f;
			apos += amove;
			a->setPosition(apos);

			float m1 = a->Mass;
			Vector2f u1 = a->Velocity;
			float m2 = 100000.f; // Infinite
			Vector2f u2 = {0,0};
			Vector2f x1 = apos;
			Vector2f x2 = p.Normal*a->getRadius()+apos;
			float masssum = m1 + m2;
			float x1subx2 = (algo::distance(x1-x2));
			float x2subx1 = (algo::distance(x2-x1));

			Vector2f v1 = u1 - (2*m2/masssum)*(algo::DotProduct(u1-u2, x1-x2)/(x1subx2*x1subx2))*(x1-x2);
			// Ground cannot move
			//Vector2f v2 = u2 - (2*m1/masssum)*(algo::DotProduct(u2-u1, x2-x1)/(x2subx1*x2subx1))*(x2-x1);
			a->Velocity = v1*0.7f;
	}
	void sovleCollisions(vector<Collision>& collisions, float dt) {
		for(Collision i: collisions) {
			Circle** a = new Circle*;
			*a = dynamic_cast<Circle*>(i.ObjA);
			Circle** b = new Circle*;
			*b = dynamic_cast<Circle*>(i.ObjB);
			Ground** c = new Ground*;
			*c = dynamic_cast<Ground*>(i.ObjA);
			Ground** d = new Ground*;
			*d = dynamic_cast<Ground*>(i.ObjB);

			if(*a && *b) {
				sovleCollision(*a,*b,i.Points,dt);
			}
			else if(*a && *d) {
				sovleCollision(*a,*d,i.Points,dt);
			}
			else if(*b && *c) {
				sovleCollision(*b,*c,i.Points,dt);
			} else {
				assert(0);
			}
			delete a;
			delete b;			
			delete c;			
			delete d;			
		}
	}
public:
	void addObject(Circle *obj) { 
		obj->setOrigin(obj->getRadius(),obj->getRadius());
		obj->Mass = obj->getRadius()*obj->getRadius()*M_PI; // TODO: mass = Area of circle
		m_objects.push_back(obj);
	}
	void addObject(Ground *obj) { 
		m_objects.push_back(obj);
	}
	// removeObject not use yet
	/*
	void removeObject(Circle *obj){
		m_objects.pop_back(); // TODO: This is wrong, add some code to find the obj needed to remove
	}
	*/
	
	void step(float dt) {
		ResolveCollisions(dt);
		for(Object* obj: m_objects) {
			Circle* c = dynamic_cast<Circle*>(obj);
			if(!c) continue;
			Vector2f pos = c->getPosition();

			// Applying F, v and g
			c->Force += c->Mass * m_gravity;
			c->Velocity += c->Force / c->Mass * dt;
			pos += c->Velocity * dt;
			c->setPosition(pos);
			c->Force = Vector2f(0,0);			
		}
		//ResolveCollisions(dt);

	}
	
	void ResolveCollisions(float dt) {
		vector<Collision> collisions;
		for(Object* a: m_objects) {
			for (Object* b: m_objects) {
				if( a==b) break;
				CollisionPoints p = a->isCollided(b);
				if(p.HasCollision) {
					collisions.emplace_back(Collision{a,b,p});
				}
			}
		}
		sovleCollisions(collisions, dt);
	}
};

//https://stackoverflow.com/questions/40629345/fill-array-dynamicly-with-gradient-color-c
unsigned int rgb(double ratio)
{
    //we want to normalize ratio so that it fits in to 6 regions
    //where each region is 256 units long
    int normalized = int(ratio * 256 * 6);

    //find the distance to the start of the closest region
    int x = normalized % 256;

    int red = 0, grn = 0, blu = 0;
    switch(normalized / 256)
    {
    case 0: red = 255;      grn = x;        blu = 0;       break;//red
    case 1: red = 255 - x;  grn = 255;      blu = 0;       break;//yellow
    case 2: red = 0;        grn = 255;      blu = x;       break;//green
    case 3: red = 0;        grn = 255 - x;  blu = 255;     break;//cyan
    case 4: red = x;        grn = 0;        blu = 255;     break;//blue
    case 5: red = 255;      grn = 0;        blu = 255 - x; break;//magenta
    }

    return ((red<<24) + (grn << 16) + (blu << 8)) | 0xff;
}

vector<Circle*> objs;
static PhysicsWorld world;

void addNewCircle(Vector2f position) {
	static double cl = 0;
	cl += 0.005; if(cl-1.0>0.001) cl = 0;
	Circle *obj = new Circle(5.f);
	obj->setPosition(position);
	obj->setFillColor(Color(rgb(cl)));

	world.addObject(obj);
	objs.push_back(obj);
}

void EventHandler(RenderWindow& window){
	static bool LeftButtonState;
	static int i=0;
	i++;
	Event event;
	while(window.pollEvent(event)) {
		if(event.type == Event::Closed) 
			window.close();
		else if(event.type == Event::KeyPressed) {
			if(Keyboard::isKeyPressed(Keyboard::Space)) {
				//
			} else if(Keyboard::isKeyPressed(Keyboard::Escape)) {
				window.close();
			} else if(Keyboard::isKeyPressed(Keyboard::W) && event.key.control){
				window.close();
			}
		} else if (event.mouseButton.button == Mouse::Left){
			LeftButtonState = !LeftButtonState;
		}
	}
	if(i>5) {
		i=0;
		Vector2f mousepos = window.mapPixelToCoords(Mouse::getPosition(window));
		if(mousepos.x <=5 || mousepos.y <=5
			|| mousepos.x >595.f || mousepos.y >595.f) LeftButtonState = false;
		if(LeftButtonState) {
			addNewCircle(window.mapPixelToCoords(Mouse::getPosition(window)));
		}
		cerr<< "Mouse = " << mousepos.x << ", " << mousepos.y << endl;
	}
}

int main() {
	ContextSettings settings;
	settings.antialiasingLevel = 8;


	RenderWindow window(VideoMode(600,600),"Free fall sim", Style::Default, settings);
	window.setFramerateLimit(60);
	//window.setVerticalSyncEnabled(true);

	// Revert the default coordinate
	// https://stackoverflow.com/questions/34310771/is-sfmlview-inverted-y-axis-standard-how-to-workaround-it
	View view = window.getDefaultView();
	view.setSize(600.f, -600.f);
	window.setView(view); 
	
	static Circle* ball = new Circle(80.f);
	ball->setPosition(100.f,400.f);	
	ball->setFillColor(Color::Green);

	Clock clock;
	Time t0= clock.getElapsedTime();

	int i=0;
	double cl = 0;
	stop = false;
	srand(time(nullptr));
	
	Ground *gnd = new Ground({1000.f,100.f});
	Ground *wall1 = new Ground({0.f,-1000.f});
	Ground *wall2 = new Ground({0.f,1000.f});
	gnd->setPosition(0.f,0);
	wall1->setPosition(5.f,1000.f);
	wall2->setPosition(500.f,0);
	world.addObject(gnd);
	world.addObject(wall1);
	world.addObject(wall2);
	world.addObject(ball); objs.push_back(ball);


	while(window.isOpen()) {
		EventHandler(window);

		// i++;
		// if(i>=10) {
		// 	i=0;
		// 	Circle *obj = new Circle(5.f);
		// 	obj->setPosition(Vector2f(50.f,500.f));
		// 	cl += 0.005; if(cl-1.0>0.001) cl = 0;
		// 	obj->setFillColor(Color(rgb(cl)));

		// 	float r1 = 20.f+ static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(50.f)));
		// 	float r2 = 30.f+ static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(50.f)));
		// 	obj->Velocity = Vector2f(r1,r2);

		// 	world.addObject(obj);
		// 	objs.push_back(obj);
		// }

		Time t1 = clock.getElapsedTime();
		Time dt = t1 - t0;
		t0=t1;

		world.step(dt.asSeconds());
		if(!stop) {
			window.clear();
			//window.draw(ball);
			window.draw(*gnd);
			window.draw(*wall1);
			window.draw(*wall2);
			for(Circle* j:objs) {
				window.draw(*j);
			}
			window.display();
		}
	}
	return EXIT_SUCCESS;
}