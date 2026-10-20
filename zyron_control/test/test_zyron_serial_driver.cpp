#include <gtest/gtest.h>
#include <zyron_control/zyron_serial_driver.hpp>

class ZyronSerialDriverTest : public testing::Test
{
public:


protected:
  zyron_control::ZyronSerialDriver serial_driver{"/dev/ttyUSB0"};
};

TEST_F(ZyronSerialDriverTest, HandleBuildRpsFrame)
{
  EXPECT_EQ(serial_driver.buildRpsFrame(12, 12), "rp12,lp12,\n");
  EXPECT_EQ(serial_driver.buildRpsFrame(-120, 120), "rn120,lp120,\n");
  EXPECT_EQ(serial_driver.buildRpsFrame(1.2, -1.2), "rp01,ln01,\n");
  EXPECT_EQ(serial_driver.buildRpsFrame(-0.4, -15), "rp00,ln15,\n");
  EXPECT_EQ(serial_driver.buildRpsFrame(0, 0), "rp00,lp00,\n");
  EXPECT_EQ(serial_driver.buildRpsFrame(0, -120), "rp00,ln120,\n");
}

TEST_F(ZyronSerialDriverTest, HandleGetParsedSerialMsg)
{
  EXPECT_EQ(serial_driver.getParsedSerialMsg("rp0.0,lp0.0,ax0.0,ay0.0,az0.0,y0.0,p0.0,r0.0"), (std::array<double, 8>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_EQ(serial_driver.getParsedSerialMsg("rp1.2,ln1.2,ax0.0,ay0.0,az0.0,y0.0,p0.0,r0.0"), (std::array<double, 8>{1.2, -1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_EQ(serial_driver.getParsedSerialMsg("rp0.4,lp15.0,ax0.0,ay0.0,az0.0,y0.0,p0.0,r0.0"), (std::array<double, 8>{0.4, 15.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
 }
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}