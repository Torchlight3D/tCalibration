#include <gtest/gtest.h>
// #include <gmock/gmock.h>

#include <tCore/AutoRegisterFactory>

// using ::testing::UnorderedElementsAre;
// using ::testing::Return;

using ::factory::Registry;

namespace test {

class Engine
{
public:
    virtual ~Engine() = default;

    virtual float consumption() const = 0;
};

class V4Engine : public Engine
{
    REGISTER("V4", Engine);

public:
    float consumption() const override { return 5.f; }
};

class V8Engine : public Engine
{
    REGISTER("V8", Engine);

public:
    float consumption() const override { return 15.f; }
};

class Vehicle
{
public:
    virtual ~Vehicle() = default;

    virtual const Engine *engine() const = 0;
    virtual int tankSize() const = 0;

    float autonomy() const
    {
        return engine() ? tankSize() / engine()->consumption() : -1.f;
    }
};

class Car : public Vehicle
{
    REGISTER("Car", Vehicle, Engine *);

public:
    explicit Car(Engine *engine) : engine_(engine) {}

    const Engine *engine() const override { return engine_; }
    int tankSize() const override { return 60; }

private:
    Engine *const engine_;
};

class Truck : public Vehicle
{
    REGISTER("Truck", Vehicle, Engine *);

public:
    explicit Truck(Engine *engine) : engine_(engine) {}

    const Engine *engine() const override { return engine_; }
    int tankSize() const override { return 140; }

private:
    Engine *const engine_;
};

class Bicycle : public Vehicle
{
    REGISTER("Bicycle", Vehicle);
    REGISTER("Motorbike", Vehicle, Engine *);

public:
    explicit Bicycle(Engine *engine = nullptr) : engine_(engine) {}

    const Engine *engine() const override { return engine_; }
    int tankSize() const override { return engine_ ? 10 : 0; }

public:
    Engine *const engine_;
};

namespace {
TEST(Engine, GetKeysWorks)
{
    EXPECT_EQ("V4", Registry<Engine>::GetKeyFor<V4Engine>());
    EXPECT_EQ("V8", Registry<Engine>::GetKeyFor<V8Engine>());
}

TEST(Vehicle, GetKeysWorks)
{
    EXPECT_EQ("Car", (Registry<Vehicle, Engine *>::GetKeyFor<Car>()));
    EXPECT_EQ("Truck", (Registry<Vehicle, Engine *>::GetKeyFor<Truck>()));
    // EXPECT_THAT((Registry<Vehicle, Engine *>::GetKeys()),
    //             UnorderedElementsAre("Car", "Truck", "Motorbike"));
    // EXPECT_THAT(Registry<Vehicle>::GetKeys(),
    // UnorderedElementsAre("Bicycle"));
}
TEST(Vehicle, MixedRegistrationAreSupported)
{
    EXPECT_EQ("Bicycle", (Registry<Vehicle>::GetKeyFor<Bicycle>()));
    EXPECT_EQ("Motorbike", (Registry<Vehicle, Engine *>::GetKeyFor<Bicycle>()));
}

} // namespace

} // namespace test

// Use a namespace to check that macros work inside another namespace.
namespace test {
namespace {

const std::string this_file(__FILE__);
const std::string deps_file =
    this_file.substr(0, this_file.size() - 3) + "_deps.cc";
//*****************************************************************************
// Test the simple case of objects with constructors taking no parameters
//*****************************************************************************
TEST(Engine, KnownKeysYieldTheRightObjects)
{
    ASSERT_TRUE(Registry<Engine>::CanNew("V4"));
    auto engine = Registry<Engine>::New("V4");
    ASSERT_TRUE(engine.get());
    EXPECT_EQ(5, engine->consumption());

    ASSERT_TRUE(Registry<Engine>::CanNew("V8"));
    auto other_engine = Registry<Engine>::New("V8");
    ASSERT_TRUE(other_engine.get());
    EXPECT_EQ(15, other_engine->consumption());
}

TEST(Engine, UnknownKeyYieldsNull)
{
    ASSERT_FALSE(Registry<Engine>::CanNew("V16"));
    auto engine = Registry<Engine>::New("V16");
    ASSERT_FALSE(engine.get());
}

// TEST(Engine, GetKeysWithLocationsWorks)
// {
//     EXPECT_THAT(Registry<Engine>::GetKeys(), UnorderedElementsAre("V4",
//     "V8")); EXPECT_THAT(Registry<Engine>::GetKeysWithLocations(),
//                 UnorderedElementsAre(deps_file + ":15: V4", //
//                                      deps_file + ":22: V8"));
// }

//*****************************************************************************
// Test advanced cases:
//  - base class takes a parameter (a vehicle receives an engine)
//  - subclass registered via multiple signatures
//*****************************************************************************

TEST(Vehicle, KnownKeysYieldTheRightObjects)
{
    auto engine = Registry<Engine>::New("V4");
    auto vehicle = Registry<Vehicle, Engine *>::New("Car", engine.get());
    ASSERT_TRUE(vehicle.get());
    EXPECT_EQ(60, vehicle->tankSize());
}

TEST(Vehicle, ObjectsCanBeInstantiatedViaMultipleRegistration)
{
    auto vehicle = Registry<Vehicle>::New("Bicycle");
    ASSERT_TRUE(vehicle.get());
    EXPECT_EQ(0, vehicle->tankSize());

    auto engine = Registry<Engine>::New("V4");
    auto other_vehicle =
        Registry<Vehicle, Engine *>::New("Motorbike", engine.get());
    ASSERT_TRUE(other_vehicle.get());
    EXPECT_EQ(10, other_vehicle->tankSize());
}

// TEST(Vehicle, GetKeysWithLocationsWorks)
// {
//     EXPECT_THAT((Registry<Vehicle, Engine *>::GetKeysWithLocations()),
//                 UnorderedElementsAre(deps_file + ":30: Car",   //
//                                      deps_file + ":43: Truck", //
//                                      deps_file + ":57: Motorbike"));
//     EXPECT_THAT(Registry<Vehicle>::GetKeysWithLocations(),
//                 UnorderedElementsAre(deps_file + ":56: Bicycle"));
// }

//*****************************************************************************
// Test ability to override registered class using an injector.
//*****************************************************************************
// class MockEngine : public Engine
// {
// public:
//     MOCK_CONST_METHOD0(consumption, float());
// };

// TEST(Registry, Injector)
// {
//     MockEngine *mock = new ::testing::NiceMock<MockEngine>();
//     ON_CALL(*mock, consumption()).WillByDefault(Return(123));

//     Registry<Engine>::Injector injector("V4", [mock]() { return mock; });

//     // Because of the injector, the New() vall below will return the mock
//     //  (taking ownership) instead of creating a new V4Engine instance.
//     std::unique_ptr<Engine> engine = Registry<Engine>::New("V4");
//     ASSERT_TRUE(engine.get());
//     EXPECT_EQ(123, engine->consumption()); // It's the mock expectation.
// }

TEST(Registry, Aliases)
{
    REGISTER_ALIAS(Vehicle, "Bicycle", "Bike");
    REGISTER_ALIAS(Vehicle, "Bicycle", "Velo");

    ASSERT_TRUE((Registry<Vehicle>::CanNew("Bike")));
    auto vehicle = Registry<Vehicle>::New("Bike");
    ASSERT_TRUE(vehicle.get());
    EXPECT_EQ(0, vehicle->tankSize());

    // EXPECT_THAT(Registry<Vehicle>::GetKeys(), ::testing::Contains("Bike*"));
    // EXPECT_THAT(Registry<Vehicle>::GetKeysWithLocations(),
    //             ::testing::Contains(this_file + ":102: Bike*"));
}

//*****************************************************************************
// Miscelleanuous tests
//*****************************************************************************
class Base
{
public:
    virtual ~Base() {}
    virtual int value() const = 0;
};

class UnregisteredDerived : public Base
{
public:
    int value() const override { return 3; }
};

class RegisteredDerived : public Base
{
public:
    REGISTER("Derived", Base);

private:
    int value() const override { return 3; }
};

class RegisteredSubDerived : public RegisteredDerived
{
public:
    REGISTER("SubDerived", Base);
    int value() const override { return 5; }
};

TEST(RegisterMacro, DoesNotChangeSize)
{
    EXPECT_EQ(sizeof(UnregisteredDerived), sizeof(RegisteredDerived));
}

TEST(RegisterMacro, WorksInHierarchies)
{
    ASSERT_TRUE(Registry<Base>::CanNew("Derived"));
    auto derived = Registry<Base>::New("Derived");
    ASSERT_TRUE(derived.get());
    EXPECT_EQ(3, derived->value());

    ASSERT_TRUE(Registry<Base>::CanNew("SubDerived"));
    auto sub_derived = Registry<Base>::New("SubDerived");
    ASSERT_TRUE(sub_derived.get());
    EXPECT_EQ(5, sub_derived->value());
}

} // namespace
} // namespace test
