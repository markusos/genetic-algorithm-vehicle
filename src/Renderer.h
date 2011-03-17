#ifndef __RENDERER_H__
#define __RENDERER_H__

#include <Box2D/Box2D.h>

namespace GA_VEHICLE
{
	class Renderer : public b2DebugDraw
	{
	public:
		Renderer(b2World* world);
		void reshape(int w, int h);
		void display(b2Body* body);
		void initGL();
	protected:
		

		int m_windowWidth;
		int m_windowHeight;
		float m_zoom;
		b2World* m_world;

		void DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color);
		void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color);
		void DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color);
		void DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color);
		void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color);
		void DrawTransform(const b2Transform& xf);
		void DrawPoint(const b2Vec2& p, float32 size, const b2Color& color);
		void DrawString(int x, int y, const char* string, ...); 
		void DrawAABB(b2AABB* aabb, const b2Color& color);
	};
}
#endif