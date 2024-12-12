import React, { useState, useEffect } from 'react';
import {
  Box,
  Button,
  Card,
  CardContent,
  CardMedia,
  Container,
  Grid,
  Typography,
  IconButton,
  Alert,
  Snackbar,
} from '@mui/material';
import { Home, Leaf, Droplet } from 'lucide-react';

const url = 'http://172.20.111.126:6969';

export default function App() {
  const [plants, setPlants] = useState([]);
  const [openSnackbar, setOpenSnackbar] = useState(false);
  const [snackbarMessage, setSnackbarMessage] = useState('');

  useEffect(() => {
    const fetchPlants = () => {
      fetch(`${url}/get_plants`)
        .then((response) => response.json())
        .then((data) => {
          // data type: [fiducial_id, plant_type, base_64_image]
          setPlants(data);
        })
        .catch((error) => {
          console.error('Error fetching plants:', error);
          showNotification('Failed to fetch plants');
        });
    };

    fetchPlants(); // Initial fetch
    const intervalId = setInterval(fetchPlants, 3000); // Fetch every 3 seconds

    return () => clearInterval(intervalId); // Cleanup on unmount
  }, []);

  const safeFetch = async (func, params) => {
    try {
      const response = await fetch(`${url}/get_instruction`);
      const res = await response.json();

      if (res.instruction === 'NONE') {
        await func(params); // Add await in case func is async
      } else {
        showNotification('Robot is busy');
      }
    } catch (error) {
      showNotification('Failed to fetch robot status');
    }
  };
  const handleBackToBase = () => {
    console.log('Sending instruction to go home');
    fetch(`${url}/update_instruction`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ instruction: 'GO_HOME' }),
    })
      .then(() => showNotification('Robot is returning to base'))
      .catch(() => showNotification('Failed to send robot home'));
  };

  const safeHandleBackToBase = () => {
    safeFetch(handleBackToBase);
  };

  const handleDetectPlants = () => {
    console.log('Detecting plants');
    fetch(`${url}/update_instruction`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ instruction: 'SCAN_ALL' }),
    })
      .then(() => showNotification('Detecting plants'))
      .catch(() => showNotification('Failed to detect plants'));
  };

  const safeHandleDetectPlants = () => {
    safeFetch(handleDetectPlants);
  };

  const handleSprayPlant = (fiducialId) => {
    console.log(`Spraying plant ${fiducialId}`);
    showNotification(`Spraying plant ${fiducialId}`);

    fetch(`${url}/update_instruction`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ instruction: `GO_TO_PLANT_${fiducialId}` }),
    })
      .then(() =>
        showNotification(
          `Going to ${fiducialId}, plant: ${
            plants.find((plant) => plant[0] === fiducialId)[1]
          }`
        )
      )
      .catch(() => showNotification(`Failed to spray plant ${fiducialId}`));
  };

  const safeHandleSprayPlant = (fiducialId) => {
    safeFetch(handleSprayPlant, fiducialId);
  };

  const showNotification = (message) => {
    setSnackbarMessage(message);
    setOpenSnackbar(true);
  };

  const resetRobot = () => {
    fetch(`${url}/update_instruction`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ instruction: 'NONE' }),
    })
      .then(() => showNotification('Robot reset'))
      .catch(() => showNotification('Failed to reset robot'));
  };

  const isMobile = window.innerWidth < 600;

  return (
    <Box
      sx={{
        minHeight: '100vh',
        bgcolor: 'background.default',
        py: 4,
      }}
    >
      <Container maxWidth="lg">
        {/* Header */}
        {isMobile ? (
          <Box marginBottom={3}>
            <Typography
              variant="h4"
              component="h1"
              sx={{
                fontWeight: 'bold',
                color: 'primary.main',
                textAlign: 'center',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
              }}
            >
              <>
                <Leaf size={40} color="#6CAF45" />
              </>
              Plant Care
            </Typography>
            <Box
              sx={{
                display: 'flex',
                justifyContent: 'center',
                gap: 2,
                mt: 2,
              }}
            >
              <Button
                variant="contained"
                color="warning"
                onClick={resetRobot}
                sx={{
                  borderRadius: 3,
                  textTransform: 'none',
                  fontSize: '1rem',
                }}
              >
                Reset
              </Button>

              <Button
                variant="contained"
                color="secondary"
                // startIcon={<Leaf />}
                onClick={safeHandleDetectPlants}
                sx={{
                  px: 4.5,
                  py: 1.5,
                  borderRadius: 3,
                  textTransform: 'none',
                  fontSize: '1rem',
                }}
              >
                Detect
              </Button>

              <Button
                variant="contained"
                color="primary"
                // startIcon={<Home />}
                onClick={safeHandleBackToBase}
                sx={{
                  // px: 1.5,
                  // py: 1.5,
                  borderRadius: 3,
                  textTransform: 'none',
                  fontSize: '1rem',
                }}
              >
                {/* Base */}
                <Home />
              </Button>
            </Box>
          </Box>
        ) : (
          <Box
            sx={{
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
              mb: 4,
            }}
          >
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
              <Leaf size={40} color="#6CAF45" />
              <Typography
                variant="h4"
                component="h1"
                sx={{
                  fontWeight: 'bold',
                  color: 'primary.main',
                }}
              >
                Plant Care Robot
              </Typography>
            </Box>
            <Box sx={{ display: 'flex', gap: 2 }}>
              <Button
                variant="contained"
                color="warning"
                onClick={resetRobot}
                sx={{
                  borderRadius: 3,
                  textTransform: 'none',
                  fontSize: '1rem',
                }}
              >
                Reset Robot
              </Button>

              <Button
                variant="contained"
                color="secondary"
                startIcon={<Leaf />}
                onClick={safeHandleDetectPlants}
                sx={{
                  px: 4,
                  py: 1.5,
                  borderRadius: 3,
                  textTransform: 'none',
                  fontSize: '1rem',
                }}
              >
                Detect Plants
              </Button>

              <Button
                variant="contained"
                color="primary"
                startIcon={<Home />}
                onClick={safeHandleBackToBase}
                sx={{
                  px: 4,
                  py: 1.5,
                  borderRadius: 3,
                  textTransform: 'none',
                  fontSize: '1rem',
                }}
              >
                Return to Base
              </Button>
            </Box>
          </Box>
        )}

        {/* Plants Grid */}
        <Grid container spacing={3}>
          {plants.map((plant) => {
            const fiducialId = plant[0];
            const plantType = plant[1];
            const base64Image = plant[2];

            return (
              <Grid item xs={12} sm={6} md={4} key={fiducialId}>
                <Card
                  sx={{
                    height: '100%',
                    display: 'flex',
                    flexDirection: 'column',
                    borderRadius: 4,
                    boxShadow: '0 4px 12px rgba(0,0,0,0.1)',
                    transition: 'transform 0.2s, box-shadow 0.2s',
                    '&:hover': {
                      boxShadow: '0 8px 24px rgba(0,0,0,0.15)',
                    },
                  }}
                >
                  <CardMedia
                    component="img"
                    height="200"
                    image={`data:image/jpeg;base64,${base64Image}`}
                    alt={plantType}
                    sx={{ objectFit: 'cover' }}
                  />
                  <CardContent sx={{ flexGrow: 1 }}>
                    <Box
                      sx={{
                        display: 'flex',
                        justifyContent: 'space-between',
                        alignItems: 'center',
                      }}
                    >
                      <Box>
                        <Typography variant="h6" gutterBottom>
                          {plantType}
                        </Typography>
                        <Typography variant="body2" color="text.secondary">
                          ID: {fiducialId}
                        </Typography>
                      </Box>
                      <IconButton
                        color="primary"
                        onClick={() => safeHandleSprayPlant(fiducialId)}
                        sx={{
                          bgcolor: 'primary.light',
                          '&:hover': { bgcolor: 'primary.main' },
                        }}
                      >
                        <Droplet size={24} color="white" />
                      </IconButton>
                    </Box>
                  </CardContent>
                </Card>
              </Grid>
            );
          })}
        </Grid>

        {/* No plants message */}
        {plants.length === 0 && (
          <Box
            sx={{
              textAlign: 'center',
              mt: 8,
              p: 4,
              bgcolor: 'background.paper',
              borderRadius: 4,
            }}
          >
            <Typography variant="h6" color="text.secondary">
              No plants detected
            </Typography>
            <Typography variant="body2" color="text.secondary" sx={{ mt: 1 }}>
              The robot will scan for plants when active
            </Typography>
          </Box>
        )}

        {/* Notification */}
        <Snackbar
          open={openSnackbar}
          autoHideDuration={4000}
          onClose={() => setOpenSnackbar(false)}
          anchorOrigin={{ vertical: 'bottom', horizontal: 'center' }}
        >
          <Alert
            onClose={() => setOpenSnackbar(false)}
            severity="success"
            sx={{ width: '100%' }}
          >
            {snackbarMessage}
          </Alert>
        </Snackbar>
      </Container>
    </Box>
  );
}
