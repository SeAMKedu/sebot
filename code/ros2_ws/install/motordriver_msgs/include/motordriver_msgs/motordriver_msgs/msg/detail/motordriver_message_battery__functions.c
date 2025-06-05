// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motordriver_msgs:msg/MotordriverMessageBattery.idl
// generated code does not contain a copyright notice
#include "motordriver_msgs/msg/detail/motordriver_message_battery__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
motordriver_msgs__msg__MotordriverMessageBattery__init(motordriver_msgs__msg__MotordriverMessageBattery * msg)
{
  if (!msg) {
    return false;
  }
  // encoder1
  // encoder2
  // speed1
  // speed2
  // pwm1
  // pwm2
  // battery
  return true;
}

void
motordriver_msgs__msg__MotordriverMessageBattery__fini(motordriver_msgs__msg__MotordriverMessageBattery * msg)
{
  if (!msg) {
    return;
  }
  // encoder1
  // encoder2
  // speed1
  // speed2
  // pwm1
  // pwm2
  // battery
}

bool
motordriver_msgs__msg__MotordriverMessageBattery__are_equal(const motordriver_msgs__msg__MotordriverMessageBattery * lhs, const motordriver_msgs__msg__MotordriverMessageBattery * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // encoder1
  if (lhs->encoder1 != rhs->encoder1) {
    return false;
  }
  // encoder2
  if (lhs->encoder2 != rhs->encoder2) {
    return false;
  }
  // speed1
  if (lhs->speed1 != rhs->speed1) {
    return false;
  }
  // speed2
  if (lhs->speed2 != rhs->speed2) {
    return false;
  }
  // pwm1
  if (lhs->pwm1 != rhs->pwm1) {
    return false;
  }
  // pwm2
  if (lhs->pwm2 != rhs->pwm2) {
    return false;
  }
  // battery
  if (lhs->battery != rhs->battery) {
    return false;
  }
  return true;
}

bool
motordriver_msgs__msg__MotordriverMessageBattery__copy(
  const motordriver_msgs__msg__MotordriverMessageBattery * input,
  motordriver_msgs__msg__MotordriverMessageBattery * output)
{
  if (!input || !output) {
    return false;
  }
  // encoder1
  output->encoder1 = input->encoder1;
  // encoder2
  output->encoder2 = input->encoder2;
  // speed1
  output->speed1 = input->speed1;
  // speed2
  output->speed2 = input->speed2;
  // pwm1
  output->pwm1 = input->pwm1;
  // pwm2
  output->pwm2 = input->pwm2;
  // battery
  output->battery = input->battery;
  return true;
}

motordriver_msgs__msg__MotordriverMessageBattery *
motordriver_msgs__msg__MotordriverMessageBattery__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motordriver_msgs__msg__MotordriverMessageBattery * msg = (motordriver_msgs__msg__MotordriverMessageBattery *)allocator.allocate(sizeof(motordriver_msgs__msg__MotordriverMessageBattery), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motordriver_msgs__msg__MotordriverMessageBattery));
  bool success = motordriver_msgs__msg__MotordriverMessageBattery__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motordriver_msgs__msg__MotordriverMessageBattery__destroy(motordriver_msgs__msg__MotordriverMessageBattery * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motordriver_msgs__msg__MotordriverMessageBattery__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motordriver_msgs__msg__MotordriverMessageBattery__Sequence__init(motordriver_msgs__msg__MotordriverMessageBattery__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motordriver_msgs__msg__MotordriverMessageBattery * data = NULL;

  if (size) {
    data = (motordriver_msgs__msg__MotordriverMessageBattery *)allocator.zero_allocate(size, sizeof(motordriver_msgs__msg__MotordriverMessageBattery), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motordriver_msgs__msg__MotordriverMessageBattery__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motordriver_msgs__msg__MotordriverMessageBattery__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
motordriver_msgs__msg__MotordriverMessageBattery__Sequence__fini(motordriver_msgs__msg__MotordriverMessageBattery__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      motordriver_msgs__msg__MotordriverMessageBattery__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

motordriver_msgs__msg__MotordriverMessageBattery__Sequence *
motordriver_msgs__msg__MotordriverMessageBattery__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motordriver_msgs__msg__MotordriverMessageBattery__Sequence * array = (motordriver_msgs__msg__MotordriverMessageBattery__Sequence *)allocator.allocate(sizeof(motordriver_msgs__msg__MotordriverMessageBattery__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motordriver_msgs__msg__MotordriverMessageBattery__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motordriver_msgs__msg__MotordriverMessageBattery__Sequence__destroy(motordriver_msgs__msg__MotordriverMessageBattery__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motordriver_msgs__msg__MotordriverMessageBattery__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motordriver_msgs__msg__MotordriverMessageBattery__Sequence__are_equal(const motordriver_msgs__msg__MotordriverMessageBattery__Sequence * lhs, const motordriver_msgs__msg__MotordriverMessageBattery__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motordriver_msgs__msg__MotordriverMessageBattery__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motordriver_msgs__msg__MotordriverMessageBattery__Sequence__copy(
  const motordriver_msgs__msg__MotordriverMessageBattery__Sequence * input,
  motordriver_msgs__msg__MotordriverMessageBattery__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motordriver_msgs__msg__MotordriverMessageBattery);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motordriver_msgs__msg__MotordriverMessageBattery * data =
      (motordriver_msgs__msg__MotordriverMessageBattery *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motordriver_msgs__msg__MotordriverMessageBattery__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motordriver_msgs__msg__MotordriverMessageBattery__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motordriver_msgs__msg__MotordriverMessageBattery__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
