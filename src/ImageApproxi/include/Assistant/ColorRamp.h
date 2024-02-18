#ifndef _COLOR_RAMP_
#define _COLOR_RAMP_

class CColorRamp
{
private :
	int m_Color[4][256];
	int m_Node[256];
	int  m_Size;
	int  m_NbNode;

public :

	// Constructor
	CColorRamp();
	~CColorRamp();

	// Data
	int GetSize() { return m_Size; }
	int Red(int index) { return m_Color[0][index]; }
	int Green(int index) { return m_Color[1][index]; }
	int Blue(int index) { return m_Color[2][index]; }

	// Misc
	int Build();
	void BuildDefault();
	int BuildNodes();
	void ResetNodes();

	void BuildRainbow();
	void BuildHsvmap();
	void BuildHotmap();
	void BuildSummer();

	void hsv2rgb(double* color ); //color[4]


};

#endif // _COLOR_RAMP_
