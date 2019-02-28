#!/usr/bin/env python3

import hebi
from time import sleep

lookup = hebi.Lookup()
sleep(2)

group = lookup.get_group_from_names(['HEBI'], ['Mobile IO'])

if group is None:
  print('Group not found: Did you forget to set the module family and names above?')
  exit(1)


def print_bank(pin_bank, bank_label):
  for i in range(0, 8):
    pin = i + 1
    if (pin_bank.has_float(pin)):
      print('Pin {0} {1} float data: {2}'.format(bank_label, pin, pin_bank.get_float(pin)))
    if (pin_bank.has_int(pin)):
      print('Pin {0} {1} int data: {2}'.format(bank_label, pin, pin_bank.get_int(pin)))


def feedback_handler(group_fbk):
  # Container to the IO feedback
  io = group_fbk.io
  print_bank(io.a, 'a')
  print_bank(io.b, 'b')
  print_bank(io.c, 'c')
  print_bank(io.d, 'd')
  print_bank(io.e, 'e')
  print_bank(io.f, 'f')


group.add_feedback_handler(feedback_handler)
group.feedback_frequency = 4.0

# Wait 10 seconds
sleep(10)

group.feedback_frequency = 0.0
group.clear_feedback_handlers()