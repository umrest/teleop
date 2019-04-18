/**
 * Author: Jason White
 *
 * Description:
 * Reads joystick/gamepad events and displays them.
 *
 * Compile:
 * gcc joystick.c -o joystick
 *
 * Run:
 * ./joystick [/dev/input/jsX]
 *
 * See also:
 * https://www.kernel.org/doc/Documentation/input/joystick-api.txt
 */
#include <fcntl.h>
#include <stdio.h>
#include <linux/joystick.h>


/**
 * Reads a joystick event from the joystick device.
 *
 * Returns 0 on success. Otherwise -1 is returned.
 */
int read_event(int fd, struct js_event *event)
{
  ssize_t bytes;

  bytes = read(fd, event, sizeof(*event));

  if (bytes == sizeof(*event))
    return 0;

  /* Error, could not read full event. */
  return -1;
}

/**
 * Returns the number of axes on the controller or 0 if an error occurs.
 */
size_t get_axis_count(int fd)
{
  __u8 axes;

  if (ioctl(fd, JSIOCGAXES, &axes) == -1)
    return 0;

  return axes;
}

/**
 * Returns the number of buttons on the controller or 0 if an error occurs.
 */
size_t get_button_count(int fd)
{
  __u8 buttons;
  if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
    return 0;

  return buttons;
}

/**
 * Current state of an axis.
 */
struct axis_state {
  short x, y;
};

/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
{
  size_t axis = event->number / 2;

  if (axis < 3)
  {
    if (event->number % 2 == 0)
      axes[axis].x = event->value;
    else
      axes[axis].y = event->value;
  }

  return axis;
}

