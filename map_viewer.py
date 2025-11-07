import dev.helper_functions as hf
import matplotlib.pyplot as plt

output_file = '100325_qbc_50.txt'

hf.plot_map(output_file)

test = hf.ask_yes_no_popup("Done looking at map?")
plt.close()