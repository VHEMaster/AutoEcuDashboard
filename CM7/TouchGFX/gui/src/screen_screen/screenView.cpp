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
  static bool vis = false;
  static bool vis_prev = true;

  float progress = (sinf(HAL_GetTick() / 1000.0f * 3.1415) + 1.0f) * 0.5f;
  float progress2 = (sinf(HAL_GetTick() / 1234.0f * 3.1415) + 1.0f) * 0.5f;
  float progress3 = (sinf(HAL_GetTick() / 600.0f * 3.1415) + 1.0f) * 0.5f;

  gaugeTachometer.setValue(progress * 1000.0f);

  gaugeSpeedMeter.setValue(progress2 * 1000.0f);

  gaugeCoolantTemp.setValue(progress3 * 1000.0f);
  gaugeOilTemp.setValue(progress3 * 1000.0f);
  gaugeFuelLevel.setValue(progress3 * 1000.0f);
  gaugeOilPress.setValue(progress3 * 1000.0f);
  gaugeManifoldPressure.setValue(progress3 * 1000.0f);
  gaugeAFR.setValue(progress3 * 1000.0f);


  static uint32_t tick_last = 0;
  uint32_t tick_current = HAL_GetTick();
  float fps = 5;

  fps = 1.0f / (tick_current - tick_last) * 1000.0f;
  tick_last = tick_current;

  Unicode::snprintfFloat(textFPSBuffer, TEXTFPS_SIZE, "%.1f", fps);
  textFPS.invalidate();

  Unicode::snprintfFloat(textAFRValueBuffer, TEXTFPS_SIZE, "%.1f", 8.0f + progress3 * 6.0f);
  Unicode::snprintfFloat(textOilTempValueBuffer, TEXTFPS_SIZE, "%.1f\xB0", -20.0f + progress3 * 130.0f);
  Unicode::snprintfFloat(textCoolantTempValueBuffer, TEXTFPS_SIZE, "%.1f\xB0", -20.0f + progress3 * 130.0f);
  Unicode::snprintfFloat(textManifoldPressureValueBuffer, TEXTFPS_SIZE, "%+.2f", -0.8f + progress3 * 2.0f);
  Unicode::snprintfFloat(textOilPressureValueBuffer, TEXTFPS_SIZE, "%.2f", progress3 * 6.0f);
  Unicode::snprintfFloat(textFuelLevelValueBuffer, TEXTFPS_SIZE, "%.1f", progress3 * 60.0f);

  if(HAL_GetTick() % 10000 > 5000) {
    vis = true;
  } else {
    vis = false;
  }

  if(vis != vis_prev) {
    textAFRValue.setVisible(vis);
    textOilTempValue.setVisible(vis);
    textCoolantTempValue.setVisible(vis);
    textManifoldPressureValue.setVisible(vis);
    textOilPressureValue.setVisible(vis);
    textFuelLevelValue.setVisible(vis);

    textAFRTitle.setVisible(vis);
    textOilTempTitle.setVisible(vis);
    textCoolantTempTitle.setVisible(vis);
    textManifoldPressureTitle.setVisible(vis);
    textOilPressureTitle.setVisible(vis);
    textFuelLevelTitle.setVisible(vis);

    gaugeCoolantTemp.setVisible(!vis);
    gaugeOilTemp.setVisible(!vis);
    gaugeFuelLevel.setVisible(!vis);
    gaugeOilPress.setVisible(!vis);
    gaugeManifoldPressure.setVisible(!vis);
    gaugeAFR.setVisible(!vis);

    textAFRValue.invalidate();
    textOilTempValue.invalidate();
    textCoolantTempValue.invalidate();
    textManifoldPressureValue.invalidate();
    textOilPressureValue.invalidate();
    textFuelLevelValue.invalidate();

    textAFRTitle.invalidate();
    textOilTempTitle.invalidate();
    textCoolantTempTitle.invalidate();
    textManifoldPressureTitle.invalidate();
    textOilPressureTitle.invalidate();
    textFuelLevelTitle.invalidate();

    gaugeCoolantTemp.invalidate();
    gaugeOilTemp.invalidate();
    gaugeFuelLevel.invalidate();
    gaugeOilPress.invalidate();
    gaugeManifoldPressure.invalidate();
    gaugeAFR.invalidate();
  } else {
    if(vis) {
      textAFRValue.invalidate();
      textOilTempValue.invalidate();
      textCoolantTempValue.invalidate();
      textManifoldPressureValue.invalidate();
      textOilPressureValue.invalidate();
      textFuelLevelValue.invalidate();
    }
  }
  vis_prev = vis;


}
