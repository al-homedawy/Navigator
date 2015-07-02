/*
 * Hard code the bearing and change the compass!
*/

package com.example.lab4_201_02;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import com.example.lab4_201_02.LineGraphView;

import android.support.v7.app.ActionBarActivity;
import android.support.v4.app.Fragment;
import android.os.Bundle;
import android.view.ContextMenu;
import android.view.ContextMenu.ContextMenuInfo;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.graphics.PointF;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

class Position implements PositionListener 
{
	// A global variable of the current path
	static List<PointF> path;
	
	// Debugging variable
	static Context applicationContext;
	
	// Keep track of key positions
	static PointF origin;
	static PointF destination;
	
	// Debugging function
	@SuppressWarnings("unused")
	static void dbg ( String msg )
	{
		AlertDialog.Builder msgBox = new AlertDialog.Builder ( applicationContext );
		msgBox.setNegativeButton("Okay", new DialogInterface.OnClickListener () {
			public void onClick(DialogInterface dialog, int which) {
				// TODO Auto-generated method stub
				
			}
		});
		msgBox.setMessage(msg);
		msgBox.show ();
	}
	
	static float calcDistance ( PointF ref, PointF trg )
	{
		return (float) ( Math.sqrt( ( Math.pow(trg.x - ref.x, 2) + Math.pow(trg.y - ref.y, 2) ) ) );
	}
	
	// Calculate the shortest path
	static List<PointF> shortestPath ( MapView source, PointF firstPoint )
	{
		// Check the parameters
		if ( firstPoint == null || destination == null )
			return null;
		
		// Create a boundary variables
		float yMin = 3;
		float yMax = ( source.getHeight () / source.getScaleY() ) - yMin;
				
		// The current obstacles in the way between firstPoint/destination
		ArrayList<PointF> obstacles = new ArrayList<PointF> ();
		
		// A variable for all the obstacles
		List<InterceptPoint> walls;
		
		// Add the first point to the initial path with obstacles
		// This can be the firstPoint or destination, depending on positions
		if ( firstPoint.x < destination.x )
		{
			// Obtain the obstacles from the firstPoint to destination
			walls = source.map.calculateIntersections(firstPoint, destination);			
			obstacles.add(firstPoint);
		}
		else
		{
			// Obtain the obstacles from the firstPoint to destination
			walls = source.map.calculateIntersections(destination, firstPoint);			
			obstacles.add(destination);
		}
		
		// Convert the intercept points into points on graph
		for ( InterceptPoint Point : walls )
			obstacles.add(Point.getPoint());
		
		// Add the last point to the initial path with obstacles
		// This can be the firstPoint or destination, depending on positions
		if ( firstPoint.x > destination.x )
			obstacles.add(firstPoint);
		else
			obstacles.add(destination);
		
		// If there are already no obstacles, then
		// display that path
		if ( obstacles.size () == 2 )
		{
			// Display the path
			source.setUserPath (obstacles);
			return obstacles;
		}
		
		
		// After receiving the points, adjust each point so each point has
		// a path with no obstacles in between
		for ( int i = 0; i < obstacles.size(); i ++  )
		{		
			// Get out of the loop if we're at the last 
			// element
			if ( ( i + 1 ) == obstacles.size () )
				break;
			
			// Create variables for the reference/next
			PointF reference = obstacles.get ( i );
			PointF next = obstacles.get ( i + 1 );
			
			// Obtain the "reference"/"next" point
			if ( ( i + 1) == ( obstacles.size () - 1) )
			{
				PointF tmp = next;
				next = reference;
				reference = tmp;
			}
			
			// Variables differentiating which course to take
			boolean goUP = false;
			boolean goDWN = false;
			
			// Try to go up if its possible
			int maxIterations = (int) ( ( yMax - next.y ) / 0.5f);
			float origY = next.y;
			
			// Attempt to go up.
			for ( int j = 0; j < maxIterations; j ++ )
			{
				next.y += 0.5f;
				
				if ( j > maxIterations )
					break;
				
				if ( source.map.calculateIntersections(reference, next).size() == 0 )
				{
					// We can go up
					goUP = true;
					break;
				}
			}
			
			if ( !goUP ) 
			{
				next.y = origY;
			
				// Try to go down if possible.
				maxIterations = (int) ( ( next.y - yMin ) / 0.5f);
				
				// Attempt to go down.
				for ( int j = 0; j < maxIterations; j ++ )
				{
					next.y -= 0.5f;
					
					if ( j > maxIterations )
						break;
					
					if ( source.map.calculateIntersections(reference, next).size() == 0 )
					{
						// We can go up
						goDWN = true;
						break;
					}
				}
				
				if ( !goDWN ) 
					next.y = origY;
			}
		}
		
		// Detect for a inverted list
		boolean transverse = ( obstacles.get (0) == destination ? true : false );
				
		// Start backwards in this case
		if ( StepOrientation.stepIndex == 0 )
			if ( transverse )
				StepOrientation.stepIndex = obstacles.size () - 1;
		
		// Display the path
		source.setUserPath (obstacles);		
		return obstacles;
	}

	// Handle origin changes.
	public void originChanged(MapView source, PointF loc) 
	{				        		
		// Reset 'path'
		if ( path != null )
			path = null;
		
		// Update 'origin'
		origin = loc;
		
		// Reset information
		StepOrientation.EastCounter = 0;
		StepOrientation.NorthCounter = 0;
		StepOrientation.steps = 0;
		StepOrientation.stepSuccess = false;
		StepOrientation.stepIndex = 0;
		
		// Set the reference point
		source.setUserPoint(origin);		
		
		// If destination is already initialized,
		// find the most-efficient path
		if ( destination != null )
			path = shortestPath ( source, origin );
	}

	// Handle position changes.
	public void destinationChanged(MapView source, PointF dest) 
	{		
		// Reset 'path'
		if ( path != null )
			path = null;
				
		// Update 'destination'
		destination = dest;
		
		// Reset information
		StepOrientation.EastCounter = 0;
		StepOrientation.NorthCounter = 0;
		StepOrientation.steps = 0;
		StepOrientation.stepSuccess = false;
		StepOrientation.stepIndex = 0;
				
		// Display the most-efficient path
		if ( origin != null )
		{
			path = shortestPath ( source, origin );
			
			// Set the user to the origin
			source.setUserPoint ( origin );
		}
	}
	
	public Position ()
	{
		// Setup variables
		origin = null;
		destination = null;
		applicationContext = null;
	}
}

class StepOrientation implements SensorEventListener {
	// A variable representing the index
	// of the position we're traveling to
	// next
	static int stepIndex = 0;
	
	// Step sensitivity (trial and error)
	final static float stepRatio = 1.0f;
	static float stepSensitivity = stepRatio;
	
	// Layouts
	private TextView bearingDisplay = null;
	private TextView instructDisplay = null;
	private TextView stepDisplay = null;
	
	// Displacement Information
	static float NorthCounter = 0.0f;
	static float EastCounter = 0.0f;
	
	// Current heading 
	static float currentHeading = 0.0f;
	
	// Orientation Data
	private float mGravity [] = null;
	private float mGeomagnetic [] = null;
	
	// Pause the step
	static boolean Pause = false;
	
	// Steps
	static int steps;

	// Go through with a step
	static boolean stepSuccess = false;
	
	// Number of steps to be recorded per sequence
	private final int maxPoints = 75;
		
	// This list keeps track of all our 'Y'
	// values.
	private ArrayList<Float> listY = new ArrayList<Float> ();
	private ArrayList<Float> listZ = new ArrayList<Float> ();
	
	// This function returns the current bearing of
	// the user.
	private String locateBearing ( float bearing )
	{
		if ( bearing < 90 )
		{
			if ( bearing < ( 90 - 80 ) )
				return "N";
			else if ( bearing > (90 - 10) )
				return "E";
			else if ( bearing < (90 - 45) )
				return "NE";
			else if ( bearing > (90 - 45) )
				return "ENE";
		}
		else if ( bearing < 180 )
		{
			if ( bearing < ( 180 - 80 ) )
				return "E";
			else if ( bearing > (180 - 10) )
				return "S";
			else if ( bearing < (180 - 45) )
				return "ESE";
			else if ( bearing > (180 - 45) )
				return "SE";
		}
		else if ( bearing < 270 )
		{
			if ( bearing < ( 270 - 80 ) )
				return "S";
			else if ( bearing > (270 - 10) )
				return "W";
			else if ( bearing < (270 - 45) )
				return "SSW";
			else if ( bearing > (270 - 45) )
				return "WSW";
		}
		else if ( bearing < 360 )
		{
			if ( bearing < ( 360 - 80 ) )
				return "W";
			else if ( bearing > (360 - 10) )
				return "N";
			else if ( bearing < (360 - 45) )
				return "WNW";
			else if ( bearing > (360 - 45) )
				return "NWW";
		}
		
		// Something wrong happened?
		return "";
	}
		
	// Use our sensor value to determine if
	// the user has made a step.
	// Record all the Y values from the beginning
	// of the wave (Y=0) to the end of the wave (Y=0).
	private boolean checkForStep ( float[] values )
	{		
		// The concept of this method is to collect 
		// 'maxPoints' number of elements then analyze that
		// data to see if there was an occurrence of a step.
		if ( listY.size () < maxPoints )
		{
			listY.add(values [1]);
			listZ.add(values [2]);
		}
		// After we've collected data we analyze it
		else
		{
			// Obtain the extremities in the graph for Y
			float maxY = Collections.max ( listY );
			float minY = Collections.min ( listY );
			
			// Obtain the extremities in the graph for Y
			float maxZ = Collections.max ( listZ );
			float minZ = Collections.min ( listZ );
			
			// Clear the data after we've recorded the
			// extremities to prepare for the next set 
			// of data.
			listY.clear ();
			listZ.clear ();
			
			// This check ensures there is sufficient movement in the Y
			// direction.
			boolean YStep = ( maxY - minY ) >= 1.2 ? true : false;
			
			// This check ensures there is a significant step in the Z
			// direction.
			boolean ZStep = ( maxZ - minZ ) >= 2.5 ? true : false;
			
			if ( YStep && ZStep )			
				// Count the step
				return true;
		}
						
		return false;
	}
	
	public void onAccuracyChanged ( Sensor sensor, int accuracy )
	{
		// We don't need this for
		// now.
	}
	
	private String computeStep ()
	{
		if ( Position.path == null || 
			 Position.origin == null ||
			 Position.destination == null ||
			 MainActivity.mapView == null)
			return "Please set up your path.";
		
		
		// Get the current user position
		PointF user = MainActivity.mapView.getUserPoint ();
		
		// Check to see if the user reached the destination
		float errX = ( Math.abs( user.x - Position.destination.x ) / Position.destination.x ) * 100;
		float errY = ( Math.abs( user.y - Position.destination.y ) / Position.destination.y ) * 100;
										
		// Check to see if the user is at the destination through some other route
		// or through our advice
		if ( ( errX < 3.5f ) && 
			 ( errY < 3.5f ) )
			// Notify the user they've reached the destination
			return "You've reached the destination!";
					
		// Detect for a inverted list
		boolean transverse = ( Position.path.get (0) == Position.destination ? true : false );
				
		// Obtain the coordinates of the path we're on
		float x = Position.path.get(stepIndex).x;
		float y = Position.path.get(stepIndex).y;
		
		// Calculate error percentages
		errX = ( Math.abs( user.x - x ) / x ) * 100;
		errY = ( Math.abs( user.y - y ) / y ) * 100;
			
		// If your not at the position 
		// we'd like you to be in then 
		// we'll tell you to go there.
		if ( ( errX > 2.5f ) || 
		     ( errY > 2.5f ) )
		{			
			// Determine the amount of steps required to reach the destination
			float width = x - user.x;
			float height = y - user.y;
			
			// Divide the steps by the 'stepSensitivity' variable
			float vertical = height / stepSensitivity;
			float horizontal = width / stepSensitivity;
			
			// Determine the direction
			String north = ( vertical < 0 ? "South" : "North" );
			String east = ( horizontal < 0 ? "West" : "East" );
			
			// Determine the amount of steps
			vertical = Math.abs( vertical );
			horizontal = Math.abs( horizontal );
			
			return String.format( "Step %d) Travel %.2f %s (exactly) and %.2f %s (exactly)", 
								 (transverse == true ? Position.path.size () - 1 - stepIndex : stepIndex), 
								 vertical, north, horizontal, east );
		}
		// If you are within an error percentage and your 
		// at the destination, go to the next step
		else 
		{
			// Increment/decrement 'stepIndex'
			if ( transverse )
				--stepIndex;
			else
				++stepIndex;
			
			// Congratulate the user for 
			// reaching the step
			return "Good!";
		}
	}
	
	public void onSensorChanged(SensorEvent event) {		
		// Compute the orientation
		if ( event.sensor.getType () == Sensor.TYPE_ACCELEROMETER )
			if ( mGravity == null )
				mGravity = event.values;
		
		if ( event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION )
		{						
			// Pass the data received from the sensor
			// to our 'checkForStep' func.
			if ( checkForStep ( event.values ) )
			{
				// Increment step counter.
				if ( Position.origin != null && Position.destination != null )
				{
					// Only increment if your not paused.
					if ( !Pause )
						++ steps;
				}
				
				// Setup 'stepSuccess'.
				stepSuccess = ( Pause || (Position.origin == null && Position.destination == null) ) ? false :
																							 true;
			}
		}
		
		// Update Orientation.
		if ( event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD )
			if ( mGeomagnetic == null )
				mGeomagnetic = event.values;
					
		if ( mGravity != null && mGeomagnetic != null )
		{
			float R[] = new float [9];
			float I[] = new float [9];
							
			if ( SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic) )
			{
				// Obtain the orientation
				float orientation [] = new float [3];
				SensorManager.getOrientation ( R, orientation );
				
				orientation [0] -= Math.PI/2;
				//orientation [0] -= ( ( 9 * Math.PI ) / 180 );
				//orientation [0] = (float) (orientation [0] % ( 2 * Math.PI ));
				
				// Obtain heading in degrees.
				//float heading = (float) Math.toDegrees ( 
				//				( (orientation [0] < 0) ? (2 * Math.PI + orientation [0]) : 
				//										orientation [0] ) );
				
				// Update 'currentHeading'
				//currentHeading = orientation [0] ; //( heading );
				
				//if ( currentHeading < 0 )
				//	currentHeading += 2 * Math.PI;
				
				// Adjust the displacement respectively.
				if ( stepSuccess )
				{					
					// In case we need to restore the north/east counters for
					// a failed step (ie: through an obstacle)
					float prevNorth = NorthCounter;
					float prevEast = EastCounter;
					
					// Break the step down into components
					double sin = Math.sin ( orientation [0] );
					double cos = Math.cos ( orientation [0] );
					
					// Apply the 'stepSensitivity' variable
					sin *= stepSensitivity;
					cos *= stepSensitivity;
					
					NorthCounter += sin;
					EastCounter += cos;
					
					// Calculate 'x' and 'y' from displacement
					float x = Position.origin.x + prevEast;
					float y = Position.origin.y + prevNorth;
					
					// Check if the user is not
					// attempting to move through an obstacle
					// Set up the current position
					PointF currentPos = new PointF ();
					currentPos.set ( x, y );
					
					// Update the position
					x = Position.origin.x + EastCounter;
					y = Position.origin.y + NorthCounter;	
					
					// Set up the next potential position
					PointF nextPos = new PointF();			
					nextPos.set ( x, y );
					
					// If the step passes the test
					if ( MainActivity.mapView.map.calculateIntersections(currentPos, nextPos).size () == 0 )
					{
						MainActivity.mapView.setUserPoint(x, y);
						PointF user = new PointF ();
						user.set( x, y );						
						Position.shortestPath(MainActivity.mapView, user);
					}
					else
					{						
						// A record of much % of the step
						// is NOT in an obstacle
						float record = 0.0f;
						
						// Try to process as much of the step 
						// as possible without moving
						// into a wall
						for ( float i = 0.0f; i <= 1.0f; i += 0.1f )
						{
							// Update the position
							x = Position.origin.x + (prevEast + ((EastCounter-prevEast) * i));	
							y = Position.origin.y + (prevNorth + ((NorthCounter-prevNorth) * i));	
							
							// Set up the next potential position
							PointF attempt = new PointF();			
							attempt.set ( x, y );
							
							// If the step passes the test
							if ( MainActivity.mapView.map.calculateIntersections(currentPos, attempt).size () == 0 )
							{
								// Move the user to the latest possible position
								MainActivity.mapView.setUserPoint(x, y);
								
								// Update 'record'
								record = i;
							}
						}
						
						// Record the final displacement after
						// squeezing in as much step as possible
						NorthCounter = (prevNorth + ((NorthCounter-prevNorth) * record));
						EastCounter = (prevEast + ((EastCounter-prevEast) * record));
					}
					
					// Reset 'stepSuccess'
					stepSuccess = false;
				}
					
				// Reset orientation data
				mGravity = null;
				mGeomagnetic = null;
			}
		}
		
		// Display the current bearing
		//.setText( String.format("Current bearing %.2f %s", 
		//												Math.toDegrees( currentHeading ), 
		//												 locateBearing ( currentHeading ) ) );
		
		// Display the next step to take
		instructDisplay.setText ( computeStep () );
		
		// Display the step counter
		stepDisplay.setText ( "Number of steps " + steps );
	}
	
	public StepOrientation ( TextView bearing, TextView instructions, TextView stepLayout )
	{
		// Initialize our variables
		bearingDisplay = bearing;
		instructDisplay = instructions;
		stepDisplay = stepLayout;
		steps = 0;
	}
}

public class MainActivity extends ActionBarActivity 
{
	// A global instance of the map
	static MapView mapView;
	
	// Main-Entry point.
    @Override
    protected void onCreate(Bundle savedInstanceState) {		
    	// Setup the application
    	super.onCreate(savedInstanceState);
    	setContentView(R.layout.activity_main);
		
    	// Create an instance of MapView
        mapView = new MapView ( getApplicationContext(), 800, 650, 30, 30 );
        
    	// Register the menu for our map
    	registerForContextMenu ( mapView );
    	
    	// Load the map
        NavigationalMap map = MapLoader.loadMap ( getExternalFilesDir ( null ), 
        										  "E2-3344.svg" );
  
        // Set the map
        mapView.setMap ( map );
        
     	// Add the listener
    	mapView.addListener(new Position ());
                
        if (savedInstanceState == null) {
            getSupportFragmentManager().beginTransaction()
                    .add(R.id.container, new PlaceholderFragment())
                    .commit();
        }
    }
    
    @Override
    public void onCreateContextMenu(ContextMenu menu, View v, ContextMenuInfo menuInfo) 
    {
    	super.onCreateContextMenu(menu, v, menuInfo);
    	mapView.onCreateContextMenu(menu, v, menuInfo);
    }
    
    @Override
    public boolean onContextItemSelected(MenuItem item) {
    	return super.onContextItemSelected(item) || mapView.onContextItemSelected (item);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();
        if (id == R.id.action_settings) {
            return true;
        }
        return super.onOptionsItemSelected(item);
    }

    /**
     * A placeholder fragment containing a simple view.
     */
    public static class PlaceholderFragment extends Fragment {

        public PlaceholderFragment() {
        	
        }

        @Override
        public View onCreateView(LayoutInflater inflater, ViewGroup container,
                Bundle savedInstanceState) {
        	
        	// Create an instance to our view
            View rootView = inflater.inflate(R.layout.fragment_main, container, false);
            
            // Setup 'applicationContext'
            Position.applicationContext = rootView.getContext ();
            
            // Update the header
            TextView Credits = (TextView) rootView.findViewById (R.id.credits);
            Credits.setText("Group 2, Section 201 -Lab 4");
            
            // Obtain an instance to our layout and set orientation
            LinearLayout layout = (LinearLayout) rootView.findViewById ( R.id.layout );
            layout.setOrientation(LinearLayout.VERTICAL);
            
            // Create a bearing layout
            TextView bearing = new TextView ( rootView.getContext () );
            bearing.setText("bearing");
            
            // Create a layout for instructions
            TextView instructions = new TextView ( rootView.getContext () );
            instructions.setText("instructions");
            
            // Create a step counter for display
            TextView stepCounter = new TextView ( rootView.getContext () );
            stepCounter.setText ( "step counter" );
            
            // Step Length (meters)
            final EditText stepLength = new EditText ( rootView.getContext () );
            stepLength.setText("1.0f");
            
            // Create a Set button.
            Button Set = new Button ( rootView.getContext () );
            Set.setText ( "Set Length." );
            Set.setOnClickListener ( new OnClickListener () {
				public void onClick(View v) {
					// Set the sensitivity
            		StepOrientation.stepSensitivity = StepOrientation.stepRatio * 
            					Float.parseFloat( stepLength.getText().toString() );			
				}            	
            });
            
            // Create a Stimulate button.
            Button Stimulate = new Button ( rootView.getContext () );
            Stimulate.setText("Stimulate Walk.");
            Stimulate.setOnClickListener( new OnClickListener () {
            	public void onClick ( View v )
            	{
            		// Stimulate a step
            		StepOrientation.stepSuccess = ( Position.origin == null && 
            										Position.destination == null ) ? false : true;
            	}
            });
            
            // Create a Pause button.
            final Button Pause = new Button ( rootView.getContext () );
            Pause.setText ( "Pause" );
            Pause.setOnClickListener( new OnClickListener () {
            	public void onClick ( View v )
            	{
            		// Pause the step
            		if ( !StepOrientation.Pause )
            		{
            			StepOrientation.Pause = true;
            			Pause.setText ( "Resume" );
            		}
            		else
            		{
            			StepOrientation.Pause = false;
            			Pause.setText ( "Pause" );
            		}
            	}
            });
            
            // Create a Reset button.
            Button Reset = new Button ( rootView.getContext () );
            Reset.setText("Reset.");
            Reset.setOnClickListener( new OnClickListener () {
            	public void onClick ( View v )
            	{            		
            		// Reset information
            		Pause.setText ( "Pause" );
            		StepOrientation.Pause = false;
            		StepOrientation.stepIndex = 0;
            		StepOrientation.EastCounter = 0;
            		StepOrientation.NorthCounter = 0;
            		StepOrientation.steps = 0;
            		StepOrientation.stepSuccess = false;
                  	
            		// Check to see if the map's been initialized
            		if ( mapView != null )
            		{          
            			if ( Position.origin != null && 
            				 Position.destination != null )
            			{
            				// Reset origin
            				Position.origin.x = 0;
	            			Position.origin.y = 0;
	            			mapView.setOriginPoint(Position.origin);
	            			
	            			// Reset user position
	            			mapView.setUserPoint(Position.origin);
	            			
	            			// Reset destination
	            			Position.destination.x = 0;
            				Position.destination.y = 0;
            				mapView.setDestinationPoint(Position.destination);  
            				
            				// Draw the path
            				Position.path = Position.shortestPath ( mapView, Position.origin );
            				Position.path = null;
            				
            				// Get rid of references
            				Position.origin = null;
            				Position.destination = null;
            			}
            			// Reset individual positions
            			else if ( Position.origin != null )
            			{
            				// Reset origin
            				Position.origin.x = 0;
	            			Position.origin.y = 0;
	            			mapView.setOriginPoint(Position.origin);
	            			mapView.setUserPoint(Position.origin);
	            			Position.origin = null;
            			}
            			else if ( Position.destination != null )
            			{
            				// Reset destination
	            			Position.destination.x = 0;
            				Position.destination.y = 0;
            				mapView.setDestinationPoint(Position.destination);  
            				mapView.setUserPoint(Position.destination);
            				Position.destination = null;
            			}
            		}
            	}
            });
            
            // Add our views to the layout
            layout.addView( mapView );
            layout.addView( stepLength );
            layout.addView( Set );
            layout.addView( Stimulate );
            layout.addView( Pause );
            layout.addView( Reset );
           // layout.addView( bearing );
            layout.addView( instructions );
            layout.addView( stepCounter );
                        
            // Create an instance of our event callback's
            SensorEventListener stepAndHeading = new StepOrientation ( bearing, 
            													       instructions,
            													       stepCounter );
            
            // Request permission to manipulate sensors
            SensorManager sensorManager = (SensorManager) 
            		rootView.getContext().getSystemService ( SENSOR_SERVICE );
            
            // Create a sensor to handle the accelerometer
            Sensor accel = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
            Sensor accelTwo = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            Sensor magnetic = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
            
            // Register our sensor
            sensorManager.registerListener(stepAndHeading, accel, 
            		SensorManager.SENSOR_DELAY_FASTEST);
            sensorManager.registerListener(stepAndHeading, accelTwo, 
            		SensorManager.SENSOR_DELAY_FASTEST);
            sensorManager.registerListener(stepAndHeading, magnetic,
            		SensorManager.SENSOR_DELAY_FASTEST);
            
            // Return
            return rootView;
        }
    }
}