#include "test.h"

class FluidProperties {
public:
	float SmokeWeight;

	void setSliderInterface(SliderInterface * sliders) {
		this->sliders = sliders;
	}

	void updateProperties() {
		SmokeWeight = sliders->getSliderValue();
	}

private:
	SliderInterface* sliders;
};