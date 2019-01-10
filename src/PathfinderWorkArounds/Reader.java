package PathfinderWorkArounds;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.Arrays;

public class Reader {

	public enum Side {

		right, left

	}

	public Reader() {

	}

	public Segment[] getSegments(Side side, int traj, int step) {

		int rows = 0;
		BufferedReader reader = null;
		try {
			reader = new BufferedReader(
					new FileReader(new File(side.toString() + "trajectorystep" + step + "traj" + traj)));
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		Segment[] segments = new Segment[1024];

		String input = "";
		try {

			reader.readLine();

			while ((input = reader.readLine()) != null) {

				String[] inputArray = input.split(",");
				segments[rows] = new Segment(inputArray);
				rows++;

			}

		} catch (Exception ex) {

			ex.getMessage();

		}

		segments = Arrays.copyOf(segments, rows);

		return segments;

	}

}
