#include <gui/screen_screen/screenView.hpp>

screenView::screenView()
{

}

void screenView::setupScreen()
{
    screenViewBase::setupScreen();
}

void screenView::tearDownScreen()
{
    screenViewBase::tearDownScreen();
}

extern "C" uint32_t HAL_GetTick(void);

#include <math.h>

void screenView::functionGaugeUpdate()
{
  float progress = (sinf(HAL_GetTick() / 1000.0f * 3.1415) + 1.0f) * 0.5f;
  float progress2 = (sinf(HAL_GetTick() / 1234.0f * 3.1415) + 1.0f) * 0.5f;
  float progress3 = (sinf(HAL_GetTick() / 600.0f * 3.1415) + 1.0f) * 0.5f;

  gaugeTachometer.setValue(progress * 100.0f);

  gaugeSpeedMeter.setValue(progress2 * 100.0f);

  gaugeCoolantTemp.setValue(progress3 * 100.0f);
  gaugeOilTemp.setValue(progress3 * 100.0f);
  gaugeFuelLevel.setValue(progress3 * 100.0f);
  gaugeOilPress.setValue(progress3 * 100.0f);
  gaugeManifoldPressure.setValue(progress3 * 100.0f);


  static uint32_t tick_last = 0;
  uint32_t tick_current = HAL_GetTick();
  float fps = 5;

  fps = 1.0f / (tick_current - tick_last) * 1000.0f;
  tick_last = tick_current;

  Unicode::snprintfFloat(screenViewBase::textFPSBuffer, screenViewBase::TEXTFPS_SIZE, "%.1f", fps);
  screenViewBase::textFPS.invalidate();

}
